from glob import glob

from setuptools import find_packages, setup

package_name = "tm5_color_pick_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/xacro", glob("xacro/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wei",
    maintainer_email="wei@todo.todo",
    description="TM5-700 Gazebo camera scene with fixed color cubes.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "move_to_ready = tm5_color_pick_demo.move_to_ready:main",
            "demo_1 = tm5_color_pick_demo.demo_1:main",
            "demo_2 = tm5_color_pick_demo.demo_2:main",
            "demo_3 = tm5_color_pick_demo.demo_3:main",
            "demo_4 = tm5_color_pick_demo.demo_4:main",
            "demo_5 = tm5_color_pick_demo.demo_5:main",
            "demo_6 = tm5_color_pick_demo.demo_6:main",
            "move_to_grid_cell = tm5_color_pick_demo.move_to_grid_cell:main",
            "validate_grid_pose_groups = tm5_color_pick_demo.validate_grid_pose_groups:main",
            "planning_scene_objects = tm5_color_pick_demo.planning_scene_objects:main",
            "red_pick_place_demo = tm5_color_pick_demo.red_pick_place_demo:main",
            "cube_pick_place_demo = tm5_color_pick_demo.cube_pick_place_demo:main",
            "language_pick_place_demo = tm5_color_pick_demo.language_pick_place_demo:main",
            "reset_cube_state = tm5_color_pick_demo.reset_cube_state:main",
            "suction_grasp_manager = tm5_color_pick_demo.suction_grasp_manager:main",
        ]
    },
)
