from setuptools import find_packages, setup

package_name = "physical_ai_drawing"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/physical_ai_drawing"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/drawing_runtime.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Physical-AI-Robot-Lab maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Runtime-generated Cartesian drawing geometry and execution.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["execute_drawing = physical_ai_drawing.drawing_runtime:main"]},
)
