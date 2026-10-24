from setuptools import find_packages, setup

package_name = "vgm_task_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Sri Charan Kumba",
    maintainer_email="164357687+sri1650@users.noreply.github.com",
    description="Perception-driven manipulation task orchestration.",
    license="MIT",
    entry_points={"console_scripts": ["task_manager = vgm_task_manager.node:main"]},
)

