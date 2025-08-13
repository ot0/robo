from setuptools import find_packages, setup

package_name = "urdf_mr999"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ot0",
    maintainer_email="ot0a6u@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control = urdf_mr999.control_publisher:main",
            "subscriber = urdf_mr999.control_subscriber:main",
        ],
    },
)
