from setuptools import find_packages, setup

package_name = 'speech_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_processor = speech_processor.audio_processor:main',
            'speech_movement = speech_processor.speech_movement:main',
            'speech_control = speech_processor.speech_control:main',
            'text_speech = speech_processor.text_speech:main',
            'speech_test = speech_processor.speech_test:main',
            'speaker_test_script = speech_processor.speaker_test_script:main',
            'wake_word = speech_processor.wake_word:main',
            'location_matcher = speech_processor.location_matcher:main',
            'intent_classifier = speech_processor.intent_classifier:main',
        ],
    },
)

