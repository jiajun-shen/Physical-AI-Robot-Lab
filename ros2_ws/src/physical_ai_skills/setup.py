from setuptools import find_packages, setup

package_name = "physical_ai_skills"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/physical_ai_skills"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/verified_pick_place.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Physical-AI-Robot-Lab maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Verified manipulation skill contracts.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "verified_pick_place = physical_ai_skills.pick_place_runtime:main",
        ],
    },
)
