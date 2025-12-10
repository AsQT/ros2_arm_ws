from setuptools import setup

package_name = "stm32_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="minhquang",
    maintainer_email="minhquang@example.com",
    description="Send 6 joint angles to STM32 via serial (int32 * 1000 format)",
    license="MIT",
    entry_points={
        "console_scripts": [
            "stm32_joint_sender = stm32_bridge.stm32_joint_sender:main",
        ],
    },
)
