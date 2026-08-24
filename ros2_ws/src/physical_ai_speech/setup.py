from setuptools import find_packages, setup


package_name = "physical_ai_speech"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/physical_ai_speech"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    # Faster-Whisper is deliberately provisioned outside colcon by the pinned
    # Phase-7 script.  Declaring it here would turn every ROS build into an
    # implicit network/package-manager action.
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Physical-AI-Robot-Lab maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Local multilingual speech normalization and drawing dispatch.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["run_local_speech = physical_ai_speech.speech_pipeline:main"]},
)
