from flask import Flask, render_template, request,make_response,send_from_directory,jsonify,redirect,url_for
import os
from ruamel import yaml
import netifaces
import subprocess
import ipaddress

import socket

app = Flask(__name__,template_folder='/home/rpdzkj/1/h265encode_test/html')

# 配置文件路径
# netplan_file = '/etc/netplan/01-configs.yaml'
netplan_file = '/etc/netplan/01-network-manager-all.yaml'
pinling_yaml = '/home/rpdzkj/1/h265encode_test/exe/config.yaml'
#rtsp_yaml = '/home/rpdzkj/env/mediamtx_/mediamtx.yml'
rtsp_yaml = '/home/rpdzkj/env/mediamtx_/mediamtx.yml'

def mask_bits(subnet_mask):
    # 将点分十进制的掩码转换为二进制字符串
    binary_str = ''.join(format(int(octet), '08b') for octet in subnet_mask.split('.'))
    # 计算二进制字符串中'1'的数量，即掩码位数
    return binary_str.count('1')

@app.route('/')
def index():
    network_info = get_network_info()
    with open(pinling_yaml, 'r', encoding='utf-8') as doc:
        pinling_config = yaml.load(doc, Loader=yaml.RoundTripLoader)

    pinling_config['UDPSendIP'] = split_ip(pinling_config['UDPSendIP'])

    
    return render_template('index.html',networkInfo=network_info,pinlingConfig = pinling_config)


@app.route('/selectfiles.html')
def select_files():
    # 获取视频和图像文件列表
    video_files = []  # 用于存储视频文件列表
    image_files = []  # 用于存储图像文件列表
    # 这里应填充video_files和image_files变量，例如使用os.listdir等函数
     # 用于获取视频文件列表，假设视频文件的扩展名为.mp4, .avi等
    #video_files = ["/home/rpdzkj/video-floder/"+f for f in os.listdir("/home/rpdzkj/video-floder/") if f.endswith(('.mp4', '.avi', '.mov'))]
    video_files = ["/home/rpdzkj/"+f for f in os.listdir("/home/rpdzkj/") if f.endswith(('.mp4', '.avi', '.mov'))]
    # 用于获取图像文件列表，假设图像文件的扩展名为.jpg, .png等
    #image_files = ["/home/rpdzkj/video-floder/"+f for f in os.listdir("/home/rpdzkj/video-floder/") if f.endswith(('.jpg', '.png', '.jpeg', '.gif'))]
    image_files = ["/home/rpdzkj/"+f for f in os.listdir("/home/rpdzkj/") if f.endswith(('.jpg', '.png', '.jpeg', '.gif'))]

    return render_template('selectfiles.html', video_files=video_files, image_files=image_files)

@app.route('/<path:filename>')
def download_file(filename):
    print(filename)
    # 根据文件类型选择正确的路径
    directory = "/"

    return send_from_directory(directory=directory, path=filename)

@app.route('/format-sdcard', methods=['POST'])
def format_sdcard():
    try:
        # 确定SD卡的设备名，比如/dev/sdb。这一步需要非常小心。
        device = '/dev/sda'  # 这里替换成你的SD卡设备名。
        
    #     # 卸载设备，这是格式化前的必须步骤
        subprocess.run(['umount', device], check=True)
        
    #     # 使用mkfs命令格式化为FAT32格式，或者你需要的任何文件系统
        subprocess.run(['mkfs.vfat', '-F', '32', device], check=True)
        
    #     # 返回成功消息
        return jsonify(message='SD card formatted successfully'), 200
    except subprocess.CalledProcessError as e:
        # 返回错误消息
        return jsonify(message=f'An error occurred: {e}'), 500


@app.route('/reboot-system', methods=['POST'])
def reboot_system():
    try:
        # Perform the system reboot
        subprocess.run(['sudo', 'reboot'], check=True)
        return jsonify(message='System is rebooting...'), 200
    except subprocess.CalledProcessError as e:
        return jsonify(message=f'An error occurred: {e}'), 500


@app.route('/submit', methods=['POST'])
def submit():
    camera_ip = request.form['camera_ip']
    gateway = request.form['gateway']
    netmask = request.form['netmask']
    udp_ip = request.form['udp_ip']
    bits  =  mask_bits(netmask)

    # 要写入文件的配置内容，这里只是一个基本示例
    config = """
network:
    version: 2
    renderer: networkd
    ethernets:
        enP3p49s0:
            dhcp4: no
            addresses: [{}]
            gateway4: {}
            nameservers:
                addresses: [8.8.8.8,8.8.4.4]
""".format(camera_ip + '/'+str(bits), gateway)  # '/24' 是 CIDR 表示法，等同于 '255.255.255.0'

#     config = """
# auto lo
# iface lo inet loopback

# auto enP3p49s0
# iface enP3p49s0 inet static
# address {}
# netmask 255.255.255.0
# gateway {}
# """.format(camera_ip, gateway)  # '/24' 是 CIDR 表示法，等同于 '255.255.255.0'

    # 编辑网络配置文件
    with open(netplan_file, 'w') as file:
        file.write(config)

    with open(pinling_yaml, 'r', encoding='utf-8') as doc:
        pinling_config = yaml.load(doc, Loader=yaml.RoundTripLoader)

    # 修改YAML中的值
    pinling_config['cameraUDPContorlPort'] = int(request.form['cameraUDPContorlPort'])
    pinling_config['videoCompressionQuality'] = request.form['videoCompressionQuality']
    pinling_config['saveFileType'] = request.form['saveFileType']
    pinling_config['HDMIOutputFPS'] = int(request.form['HDMIOutputFPS'])
    pinling_config['streamType'] = request.form['streamType']
    pinling_config['resolution'] = request.form['resolution']
    pinling_config['RTSPServerPort'] = int(request.form['RTSPServerPort'])
    pinling_config['RTSPEncoderBitrate'] = int(request.form['RTSPEncoderBitrate'])
    pinling_config['webPort'] = int(request.form['webPort'])
    pinling_config['RTSPOutputForImageTransmission'] = request.form['RTSPOutputForImageTransmission']
    pinling_config['OSDSrtSet'] = request.form['OSDSrtSet']
    pinling_config['recognitionClass'] = request.form['recognitionClass']
    pinling_config['UDPSendSwitch'] = request.form['UDPSendSwitch']
    pinling_config['UDPSendPort'] = int(request.form['UDPSendPort'])
    pinling_config['UDPSendType'] = request.form['UDPSendType']
    pinling_config['UDPSendIP'] = udp_ip




    # 保存修改后的YAML文件
    with open(pinling_yaml, 'w',encoding='utf-8') as file:
        yaml.dump(pinling_config, file,Dumper=yaml.RoundTripDumper)

    with open(rtsp_yaml, 'r', encoding='utf-8') as file:
        lines = file.readlines()

    with open(rtsp_yaml, 'w', encoding='utf-8') as file:
        for line in lines:
            if line.strip().startswith('rtspAddress:'):
                # 替换整行
                file.write('rtspAddress: :' + request.form['RTSPServerPort']+ '\n')
            else:
                # 原样写回
                file.write(line)
    return redirect(url_for('index'))


def split_ip(ip):
    return ip.split('.')
def get_network_info():
    # 获取所有网络接口（字典）
    ifaces = netifaces.interfaces()
    
    # 网络详细信息
    network_info = {}
    
    # 遍历所有网络接口
    for iface in ifaces:
        if iface == "enP3p49s0":
            addrs = netifaces.ifaddresses(iface)
            # 获取AF_INET属性
            if netifaces.AF_INET in addrs:
                # ip_info包含IP地址和子网掩码
                ip_info = addrs[netifaces.AF_INET][0]
                address = ip_info['addr']
                netmask = ip_info['netmask']
                
                # 获取默认网关
                try:
                    gateways = netifaces.gateways()
                    default_gateway = gateways['default'][netifaces.AF_INET][0]
                except KeyError:
                    default_gateway = None

                network_info["interface"] = iface
                network_info["address"] = split_ip(address)
                network_info["netmask"] = split_ip(netmask)
                network_info["gateway"] = split_ip(default_gateway)
    return network_info

if __name__ == '__main__':

    with open(pinling_yaml, 'r', encoding='utf-8') as doc:
        pinling_config = yaml.load(doc, Loader=yaml.RoundTripLoader)
    

    app.run(host="0.0.0.0",port=pinling_config['webPort'],debug=True)
