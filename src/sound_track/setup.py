from setuptools import setup

package_name = "sound_track"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/sound_track.launch.py", "launch/sound_track_serial.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wheeltec",
    maintainer_email="maintainer@example.com",
    description="Sound source tracking helper that publishes /awake_angle",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "angle_publisher = sound_track.angle_publisher:main",
            "serial_angle_node = sound_track.serial_angle_node:main",
            "sound_track_controller = sound_track.controller_node:main",
        ],
    },
)
