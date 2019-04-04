from setuptools import setup

package_name = 'rosshow2'
setup(
    name=package_name,
    version='0.6.2',
    package_dir={"": "src"},
    packages=["librosshow2", "librosshow2.viewers", "librosshow2.viewers.generic", "librosshow2.viewers.sensor_msgs"],
    install_requires=['setuptools'],
    zip_safe=True,
    scripts=['nodes/rosshow2']
)
