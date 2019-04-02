from setuptools import setup

package_name = 'rosshow'
setup(
    name=package_name,
    version='0.6.2',
    package_dir={"": "src"},
    packages=["librosshow", "librosshow.viewers", "librosshow.viewers.generic", "librosshow.viewers.sensor_msgs"],
    install_requires=['setuptools'],
    zip_safe=True,
    scripts=['nodes/rosshow']
)
