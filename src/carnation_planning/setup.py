from setuptools import find_packages, setup


package_name = "carnation_planning"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/simple_alignment.launch.py"]),
        (f"share/{package_name}/config", ["config/simple_target_follower.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@example.com",
    description="Minimal target-to-joint planning package for simplified carnation robot simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "simple_target_follower = carnation_planning.simple_target_follower:main",
        ],
    },
)
