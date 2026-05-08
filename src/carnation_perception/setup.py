from setuptools import find_packages, setup


package_name = "carnation_perception"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/perception_mvp.launch.py"]),
        (
            "share/" + package_name + "/config",
            [
                "config/image_source.yaml",
                "config/yolo_detector.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@example.com",
    description="Perception nodes for carnation detection and pixel center extraction.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_source_node = carnation_perception.image_source_node:main",
            "yolo_detector_node = carnation_perception.yolo_detector_node:main",
        ],
    },
)
