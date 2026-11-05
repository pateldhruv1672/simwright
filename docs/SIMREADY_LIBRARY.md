# Native SimReady library

The Isaac Sim extension now has a collapsible library browser with background search, metric dimensions, pagination and Add to scene through the existing agent/approval queue. POST /assets/search is authenticated and handled outside Kit. Searches cover the public SimReady Central index across publishers, not just the local cache.

Imports use simready_fetch/simready_place: complete revision-pinned packages with materials, textures and physics, downloaded outside Kit before local USD references are authored. No blanket library download is necessary. Upstream availability and access policies apply.

Validation: 47 tests passed, extension source compiles, live catalog search succeeded. Native omni.ui interaction still needs verification after upgrading the PC package, restarting the panel server and reloading the extension. No user scene was modified.
