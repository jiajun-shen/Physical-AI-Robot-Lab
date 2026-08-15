from setuptools import find_packages, setup
package_name = "physical_ai_speech"
setup(name=package_name, version="0.1.0", packages=find_packages(exclude=["test"]), data_files=[("share/ament_index/resource_index/packages", ["resource/physical_ai_speech"]), (f"share/{package_name}", ["package.xml"])], install_requires=["setuptools"], zip_safe=False, maintainer="Physical-AI-Robot-Lab maintainers", maintainer_email="maintainers@example.invalid", description="Multilingual speech normalization contracts.", license="Apache-2.0", tests_require=["pytest"])
