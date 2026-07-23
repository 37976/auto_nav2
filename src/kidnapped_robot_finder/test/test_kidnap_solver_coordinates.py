import numpy as np

from global_localizer.kidnap_solver import _map_pixel_to_world


def test_map_pixel_to_world_uses_pixel_centers():
    world = _map_pixel_to_world(
        pixel_uv=(0.0, 0.0),
        map_height_px=100,
        map_resolution=0.05,
        map_origin=(-2.0, -3.0),
    )

    np.testing.assert_allclose(world, (-1.975, 1.975))


def test_map_pixel_to_world_preserves_subpixel_refinement():
    world = _map_pixel_to_world(
        pixel_uv=(39.25, 69.75),
        map_height_px=100,
        map_resolution=0.05,
        map_origin=(-2.0, -3.0),
    )

    np.testing.assert_allclose(world, (-0.0125, -1.5125))


def test_map_pixel_to_world_applies_map_specific_calibration():
    world = _map_pixel_to_world(
        pixel_uv=(0.0, 0.0),
        map_height_px=100,
        map_resolution=0.05,
        map_origin=(-2.0, -3.0),
        map_pose_offset=(0.0486, -0.0639),
    )

    np.testing.assert_allclose(world, (-1.9264, 1.9111))
