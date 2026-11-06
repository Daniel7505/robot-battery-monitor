"""Onboard 2D mapping. No apartment geometry.

The Webots process that imports this package is ``apartment_mapper``.
A separate pose logger, not imported here, records the simulator pose
for scoring after the run. The explorer can be handed that pose for
planning. It does not read the logger, and the grid stays on the estimate.
"""
