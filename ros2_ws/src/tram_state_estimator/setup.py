from setuptools import setup

package_name = "tram_state_estimator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/estimator.launch.py",
                                                "launch/tram.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml",
                                                "config/tram.yaml",
                                                "config/track_map.npz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lab9",
    maintainer_email="lab9@bitcoff.io",
    description="Резервная навигация трамвая без GNSS",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tram_estimator = tram_state_estimator.tram_node:main",
            "estimator_node = tram_state_estimator.estimator_node:main",
            "simulator_node = tram_state_estimator.simulator_node:main",
        ],
    },
)
