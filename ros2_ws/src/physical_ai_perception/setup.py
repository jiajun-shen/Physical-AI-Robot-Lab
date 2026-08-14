from setuptools import find_packages, setup


package_name = "physical_ai_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/physical_ai_perception"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/vision_classes.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Physical-AI-Robot-Lab maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Vision-only RGB-D object localization for Physical-AI-Robot-Lab.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_localizer_node = physical_ai_perception.vision_localizer_node:main",
        ],
    },
)
