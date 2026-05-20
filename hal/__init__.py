"""Hardware Abstraction Layer.

Each module exposes a class that owns its hardware resource and offers a
clean Python API to the rest of the codebase. All trace events go through
``logs.trace.tracer`` so reaction-delay measurement comes for free.
"""
