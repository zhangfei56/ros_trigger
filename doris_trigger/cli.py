import argparse
import os
import json

home_dir = os.path.expanduser('~')
config_dir = os.path.join(home_dir, '.doris', 'trigger')


def read_config(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)

def main():
    parser = argparse.ArgumentParser(description='Doris Trigger CLI')
    
    # 增加 check update 参数
    parser.add_argument('--update', action='store_true', help='检查更新')
    
    parser.add_argument('--immediate', action='store_true', help='立即触发')
        
    config_path = os.path.join(config_dir, 'rule_engine.json')
    parser.add_argument('--config', type=str, default=config_path, help='指定配置文件.')
    
    log_dir = os.path.join(config_dir, 'logs')
    parser.add_argument('--log_dir', type=str, default=log_dir, help='指定日志文件.')
    
    
    args = parser.parse_args()
    
    if args.update:
        print('检查更新中...')
        return
    
    # 检查配置文件
    if not os.path.exists(args.config_path):
        print('配置文件不存在，请重现安装，或者指定配置文件.')
        return
    # 读取配置文件
    try:
        config_json = read_config(args.config)
    except Exception as e:
        print(f'配置文件格式错误，请检查配置文件. \n {e}')
        return
    
    



if __name__ == '__main__':
    main()
