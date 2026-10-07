import pathlib
import unittest


class LaunchSyntaxTest(unittest.TestCase):
    def test_launch_files_compile(self):
        launch_dir = pathlib.Path(__file__).parents[1] / "launch"
        for launch_file in sorted(launch_dir.glob("*.launch.py")):
            with self.subTest(launch_file=launch_file.name):
                compile(launch_file.read_text(), str(launch_file), "exec")


if __name__ == "__main__":
    unittest.main()
