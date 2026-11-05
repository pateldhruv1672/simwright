import os, pathlib, pytest

@pytest.fixture(autouse=True)
def home(tmp_path, monkeypatch):
    h = tmp_path / "swhome"
    monkeypatch.setenv("SIMWRIGHT_HOME", str(h))
    import importlib, simwright.config as c
    c.HOME = h
    yield h
