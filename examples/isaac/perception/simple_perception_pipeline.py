"""
Simple perception pipeline for Isaac Sim.
This example demonstrates how to process sensor data in Isaac Sim.
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.viewports import get_viewport_from_window_name
import numpy as np
import carb
import cv2


class SimplePerceptionPipeline:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.camera = None
        self.assets_root_path = get_assets_root_path()

        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets. Ensure Isaac Sim is properly installed.")
            return

        # Add a simple robot to the scene
        self._setup_scene()

        # Setup camera for perception
        self._setup_camera()

    def _setup_scene(self):
        """Setup the scene with a simple robot."""
        # Add ground plane
        add_reference_to_stage(
            usd_path=self.assets_root_path + "/Isaac/Props/Grid/default_unit_grid.usd",
            prim_path="/World/grid"
        )

        # Add a simple robot - using Carter as example
        add_reference_to_stage(
            usd_path=self.assets_root_path + "/Isaac/Robots/Carter/carter_navigate.usd",
            prim_path="/World/Robot"
        )

    def _setup_camera(self):
        """Setup camera for perception tasks."""
        # Create camera prim
        self.camera = Camera(
            prim_path="/World/Robot/base_link/Camera",
            frequency=30,
            resolution=(640, 480)
        )

        # Add camera to world
        self.world.scene.add(self.camera)

    def run_perception(self):
        """Run the perception pipeline."""
        # Reset the world
        self.world.reset()

        # Run simulation and perception
        for i in range(500):  # Run for 500 steps
            self.world.step(render=True)

            # Get camera data periodically
            if i % 30 == 0:  # Every 30 steps (1 Hz)
                rgb_data = self.camera.get_rgb()

                if rgb_data is not None:
                    # Process the RGB image
                    self._process_image(rgb_data, i)

                    # Log some information
                    print(f"Processed frame {i}, RGB shape: {rgb_data.shape}")

    def _process_image(self, image, frame_num):
        """Process the camera image for perception."""
        # Convert to OpenCV format
        image_cv = image[:, :, :3]  # Take RGB channels

        # Simple processing example: detect edges
        gray = cv2.cvtColor(image_cv, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Save processed image
        cv2.imwrite(f"processed_frame_{frame_num:04d}.png",
                   cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB))

        # In a real application, you would perform more sophisticated
        # perception tasks like object detection, segmentation, etc.

    def cleanup(self):
        """Cleanup resources."""
        if self.world.physics_sim_view is not None:
            self.world.clear()
        self.world = None


def main():
    """Main function to run the perception pipeline."""
    try:
        pipeline = SimplePerceptionPipeline()
        pipeline.run_perception()
        pipeline.cleanup()
        print("Perception pipeline completed successfully!")
    except Exception as e:
        print(f"Error running perception pipeline: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()