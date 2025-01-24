from setuptools import find_packages, setup

package_name = "moon_exploration"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miguel",
    maintainer_email="mgons@unileon.es",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "moon_exploration_node = moon_exploration.moon_exploration_node:main",
        ],
    },
)
