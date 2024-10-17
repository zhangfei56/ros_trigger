import shutil
from setuptools import setup, find_packages
import os
from setuptools.command.install import install


# 当前用户目录
home_dir = os.path.expanduser('~')
config_dir = os.path.join(home_dir, '.doris', 'trigger')


install_requirements = open('requirements.txt').readlines()
ROOT = os.path.dirname(__file__)


def get_version():
    with open('VERSION', 'r') as f:
        version = f.read().strip()
    return version

class CustomInstallCommand(install):
    def run(self):
        # # 检查是否是更新
        # if self.old_and_unmanageable or self.single_version_externally_managed:
        #     print("Updating the package...")
        #     self.execute(self._pre_update, (), msg="Running pre-update operations")
        # else:
        #     print("Fresh installation...")
        #     self.execute(self._pre_install, (), msg="Running pre-install operations")
        
        # 运行实际的安装
        install.run(self)

        # 安装后的操作
        self.execute(self._post_install, (), msg="Running post-install operations")

    # def _pre_install(self):
    #     # 在这里添加安装前的操作
    #     print("Performing pre-install operations...")
    #     # check config_dir exists
    #     if not os.path.exists(config_dir):
    #         os.makedirs(config_dir)
    #         print('create config dir')

    # def _pre_update(self):
    #     # 在这里添加更新前的操作
    #     print("Performing pre-update operations...")
    #     # 例如：备份用户数据，清理旧文件等
    #     default_rule_engine_path = os.path.join(os.path.dirname(__file__), 'default_rule_engine.json')
    #     shutil.copyfile(default_rule_engine_path, os.path.join(config_dir, 'rule_engine.json'))
    #     print('copy default_rule_engine.json to config_dir')


    def _post_install(self):
        # 在这里添加安装后的操作
        print("Performing post-install operations...")

        try:
            os.makedirs(config_dir, exist_ok=True)
            print(f"Created config directory: {config_dir}")
        except OSError as e:
            print(f"Failed to create config directory: {e}")

        # 复制默认配置文件
        default_rule_engine_path = os.path.join(os.path.dirname(__file__), 'default_rule_engine.json')
        target_path = os.path.join(config_dir, 'rule_engine.json')
        try:
            shutil.copyfile(default_rule_engine_path, target_path)
            print(f"Copied default rule engine configuration to {target_path}")
        except IOError as e:
            print(f"Failed to copy default rule engine configuration: {e}")

setup(
    name='doris_trigger',
    version=get_version(),
    packages=find_packages(),
    # package_dir={'': 'src'},
    entry_points={
        'console_scripts': [
            'doris_trigger=doris_trigger.cli:main',
        ],
    },
    cmdclass={
        'install': CustomInstallCommand,
    },
    author='Nicky',
    author_email='zhangjianfei@pegasus.tech',
    description='Doris trigger data collector.',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/yourusername/my_cli_project',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    install_requires=install_requirements,
    python_requires='>=3.9',
)
