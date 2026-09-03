"""Shared type aliases used across the Fly-in simulation modules.

Hub and connection records are built up incrementally (fields are added
after the initial dict literal) and their ``metadata`` sub-dict can hold
arbitrary user-supplied keys, so ``Dict[str, Any]`` is the accurate type
for them rather than a fixed ``TypedDict``.
"""

from typing import Any, Dict, List, Tuple

HubDict = Dict[str, Any]
ConnectionDict = Dict[str, Any]

ParsedData = Tuple[
    Dict[str, HubDict],
    Dict[str, ConnectionDict],
    int,
    Dict[str, List[str]],
]
