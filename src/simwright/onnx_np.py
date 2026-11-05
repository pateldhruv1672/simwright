"""Minimal NumPy runner for exported LEAPP/ONNX policies (no onnxruntime, no torch).

Supports the operator set of AGILE's exported locomotion policies. The graph is pre-converted with
`export_onnx_graph()` (needs the `onnx` package, sandbox/dev only) to a .npz the runner loads anywhere.
"""
from __future__ import annotations
import json
import numpy as np

OPS = {"Mul", "Slice", "Reshape", "Concat", "Sub", "Unsqueeze", "ScatterND", "Transpose", "Gemm", "Split", "Elu",
       "Add", "Clip", "Max", "Min", "Div", "Relu", "Tanh", "Sigmoid", "Identity", "Gather", "Squeeze", "Neg", "Cast",
       "MatMul", "Constant", "Shape", "Expand", "Flatten", "Abs", "Sqrt", "Pow"}


def export_onnx_graph(onnx_path: str, npz_path: str) -> dict:
    import onnx
    from onnx import numpy_helper
    m = onnx.load(onnx_path)
    arrays = {f"init::{t.name}": numpy_helper.to_array(t) for t in m.graph.initializer}
    nodes = []
    for i, n in enumerate(m.graph.node):
        if n.op_type not in OPS:
            raise ValueError(f"unsupported op {n.op_type}")
        attrs = {}
        for a in n.attribute:
            v = onnx.helper.get_attribute_value(a)
            if isinstance(v, onnx.TensorProto):
                arrays[f"attr::{i}::{a.name}"] = numpy_helper.to_array(v); v = {"__tensor__": f"attr::{i}::{a.name}"}
            elif isinstance(v, bytes):
                v = v.decode()
            elif isinstance(v, (list, tuple)):
                v = [x.decode() if isinstance(x, bytes) else x for x in v]
            attrs[a.name] = v
        nodes.append({"op": n.op_type, "in": list(n.input), "out": list(n.output), "attrs": attrs})
    graph = {"nodes": nodes, "inputs": [i.name for i in m.graph.input if f"init::{i.name}" not in arrays],
             "outputs": [o.name for o in m.graph.output]}
    np.savez_compressed(npz_path, __graph__=np.frombuffer(json.dumps(graph).encode(), dtype=np.uint8), **arrays)
    return graph


class NpGraph:
    def __init__(self, npz_path: str):
        z = np.load(npz_path, allow_pickle=False)
        self.graph = json.loads(bytes(z["__graph__"]).decode())
        self.const = {k[6:]: z[k] for k in z.files if k.startswith("init::")}
        self.attr_t = {k: z[k] for k in z.files if k.startswith("attr::")}
        self.inputs, self.outputs = self.graph["inputs"], self.graph["outputs"]

    def __call__(self, feeds: dict) -> dict:
        env = dict(self.const)
        env.update({k: np.asarray(v) for k, v in feeds.items()})
        for n in self.graph["nodes"]:
            a = {k: (self.attr_t[v["__tensor__"]] if isinstance(v, dict) and "__tensor__" in v else v) for k, v in n["attrs"].items()}
            x = [env[i] if i else None for i in n["in"]]
            outs = _run(n["op"], x, a)
            for name, val in zip(n["out"], outs):
                env[name] = val
        return {o: env[o] for o in self.outputs}


def _run(op, x, a):
    if op == "Mul": return [x[0] * x[1]]
    if op == "Add": return [x[0] + x[1]]
    if op == "Sub": return [x[0] - x[1]]
    if op == "Div": return [x[0] / x[1]]
    if op == "Neg": return [-x[0]]
    if op == "Abs": return [np.abs(x[0])]
    if op == "Sqrt": return [np.sqrt(x[0])]
    if op == "Pow": return [np.power(x[0], x[1])]
    if op == "Max": return [np.maximum.reduce(x)]
    if op == "Min": return [np.minimum.reduce(x)]
    if op == "Relu": return [np.maximum(x[0], 0)]
    if op == "Tanh": return [np.tanh(x[0])]
    if op == "Sigmoid": return [1 / (1 + np.exp(-x[0]))]
    if op == "Identity": return [x[0]]
    if op == "Cast": return [x[0]]
    if op == "Constant": return [a["value"]]
    if op == "Shape": return [np.array(x[0].shape, dtype=np.int64)]
    if op == "Elu":
        al = a.get("alpha", 1.0); return [np.where(x[0] > 0, x[0], al * (np.exp(np.minimum(x[0], 0)) - 1)).astype(x[0].dtype)]
    if op == "Clip":
        lo = x[1] if len(x) > 1 and x[1] is not None else a.get("min", -np.inf)
        hi = x[2] if len(x) > 2 and x[2] is not None else a.get("max", np.inf)
        return [np.clip(x[0], lo, hi)]
    if op == "Gemm":
        A, B = x[0], x[1]
        if a.get("transA", 0): A = A.T
        if a.get("transB", 0): B = B.T
        y = a.get("alpha", 1.0) * (A @ B)
        if len(x) > 2 and x[2] is not None: y = y + a.get("beta", 1.0) * x[2]
        return [y.astype(np.float32)]
    if op == "MatMul": return [np.matmul(x[0], x[1])]
    if op == "Reshape":
        shp = [int(s) for s in x[1]]
        shp = [x[0].shape[i] if s == 0 and not a.get("allowzero", 0) else s for i, s in enumerate(shp)]
        return [x[0].reshape(shp)]
    if op == "Flatten":
        ax = a.get("axis", 1); return [x[0].reshape(int(np.prod(x[0].shape[:ax])), -1)]
    if op == "Expand": return [x[0] * np.ones([int(s) for s in x[1]], dtype=x[0].dtype)]
    if op == "Transpose":
        perm = a.get("perm"); return [np.transpose(x[0], perm)]
    if op == "Unsqueeze":
        axes = [int(v) for v in (x[1] if len(x) > 1 and x[1] is not None else a["axes"])]
        y = x[0]
        for ax in sorted(ax if ax >= 0 else ax + y.ndim + 1 for ax in axes): y = np.expand_dims(y, ax)
        return [y]
    if op == "Squeeze":
        axes = x[1] if len(x) > 1 and x[1] is not None else a.get("axes")
        return [np.squeeze(x[0], axis=tuple(int(v) for v in axes) if axes is not None else None)]
    if op == "Concat": return [np.concatenate([v for v in x if v is not None], axis=a["axis"])]
    if op == "Gather": return [np.take(x[0], x[1].astype(np.int64), axis=a.get("axis", 0))]
    if op == "Split":
        ax = a.get("axis", 0)
        if len(x) > 1 and x[1] is not None:
            idx = np.cumsum([int(s) for s in x[1]])[:-1]
        else:
            n = a.get("num_outputs"); idx = np.cumsum([x[0].shape[ax] // n] * n)[:-1]
        return list(np.split(x[0], idx, axis=ax))
    if op == "Slice":
        data, starts, ends = x[0], x[1], x[2]
        axes = x[3] if len(x) > 3 and x[3] is not None else np.arange(len(starts))
        steps = x[4] if len(x) > 4 and x[4] is not None else np.ones(len(starts), dtype=np.int64)
        sl = [slice(None)] * data.ndim
        for s, e, ax, st in zip(starts, ends, axes, steps):
            ax = int(ax) % data.ndim; dim = data.shape[ax]
            s, e, st = int(s), int(e), int(st)
            s = max(-dim - 1, min(s, dim)); e = max(-dim - 1, min(e, dim))
            sl[ax] = slice(s, e if not (st < 0 and e < -dim) else None, st)
        return [data[tuple(sl)]]
    if op == "ScatterND":
        data = np.array(x[0], copy=True); idx = x[1].astype(np.int64); upd = x[2]
        k = idx.shape[-1]
        for pos in np.ndindex(idx.shape[:-1]):
            data[tuple(idx[pos])] = upd[pos]
        return [data]
    raise NotImplementedError(op)
