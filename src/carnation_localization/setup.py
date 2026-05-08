from setuptools import find_packages, setup


package_name = "carnation_localization"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/localization_mvp.launch.py",
                "launch/vision_to_target_pose.launch.py",
                "launch/stereo_localization.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/pixel_to_world.yaml",
                "config/stereo_pixel_to_world.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@example.com",
    description="Localization nodes for converting pixel detections to target poses.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pixel_to_world_node = carnation_localization.pixel_to_world_node:main",
            "stereo_pixel_to_world_node = carnation_localization.stereo_pixel_to_world_node:main",
        ],
    },
)
