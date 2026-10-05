import pathlib
import unittest


class LaunchSyntaxTest(unittest.TestCase):
    def test_simulation_launch_compiles(self):
        launch_file = pathlib.Path(__file__).parents[1] / "launch" / "simulation.launch.py"
        compile(launch_file.read_text(), str(launch_file), "exec")


if __name__ == "__main__":
    unittest.main()
