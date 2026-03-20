from setuptools import find_packages, setup


package_name = "hakoniwa_pdu_ros"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config/sample",
            [
                "config/sample/endpoint_zenoh.json",
                "config/sample/endpoint_zenoh_connect.json",
                "config/sample/sample_binding.json",
            ],
        ),
        (f"share/{package_name}/config/sample/cache", ["config/sample/cache/buffer.json"]),
        (
            f"share/{package_name}/config/sample/comm",
            [
                "config/sample/comm/zenoh_pubsub_comm.json",
                "config/sample/comm/zenoh_pubsub_comm_connect.json",
            ],
        ),
        (
            f"share/{package_name}/config/sample/comm/zenoh",
            [
                "config/sample/comm/zenoh/peer_connect.json5",
                "config/sample/comm/zenoh/peer_listen.json5",
            ],
        ),
        (
            f"share/{package_name}/config/sample/pdu",
            ["config/sample/pdu/pdudef.json", "config/sample/pdu/pdutypes.json"],
        ),
        (
            f"share/{package_name}/examples",
            [
                "examples/README.md",
                "examples/zenoh_peer.py",
                "examples/ros_pos_subscriber.py",
                "examples/ros_cmd_publisher.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tmori",
    maintainer_email="tmori@example.com",
    description="ROS 2 bridge package for Hakoniwa PDU endpoints.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = hakoniwa_pdu_ros.__main__:main",
        ],
    },
)
