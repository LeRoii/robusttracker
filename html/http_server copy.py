from flask import Flask, render_template, request,make_response
import os
from ruamel import yaml
import netifaces

import socket

app = Flask(__name__,template_folder='/home/rpdzkj/wjm/pinlingv2.3.1/html')

# 配置文件路径
netplan_file = '/etc/netplan/01-configs.yaml'
pinling_yaml = '/home/rpdzkj/wjm/pinlingv2.3.1/exe/config.yaml'
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


@app.route('/download_file/<filename>')
def download_file(filename):
    return send_from_directory(FILES_DIRECTORY, filename, as_attachment=True)

@app.route('/submit', methods=['POST'])
def submit():
    camera_ip = request.form['camera_ip']
    gateway = request.form['gateway']
    netmask = request.form['netmask']
    udp_ip = request.form['udp_ip']
    bits  =  mask_bits(netmask)

    print(bits)

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

    # print("cameraUDPContorlPort: {}   videoCompressionQuality:{}\n \
    #       saveFileType: {}   HDMIOutputFPS:{}\n \
    #       streamType: {}   resolution:{}\n \
    #       RTSPServerPort: {}   RTSPEncoderBitrate:{}\n \
    #       webPort: {}   RTSPOutputForImageTransmission:{}\n \
    #       OSDSrtSet: {}   recognitionClass:{}\n \
    #       UDPSendIP: {}   UDPSendSwitch:{}\n \
    #       UDPSendPort: {}   UDPSendType:{}\n ".format(pinling_config['cameraUDPContorlPort'],pinling_config['videoCompressionQuality'], \
    #           pinling_config['saveFileType'],pinling_config['HDMIOutputFPS'] ,
    #           pinling_config['streamType'],pinling_config['resolution'] ,\
    #           pinling_config['RTSPServerPort'],pinling_config['RTSPEncoderBitrate'] ,\
    #           pinling_config['webPort'],pinling_config['RTSPOutputForImageTransmission'] ,\
    #           pinling_config['OSDSrtSet'],pinling_config['recognitionClass'] ,\
    #           udp_ip,pinling_config['UDPSendSwitch'] ,\
    #           pinling_config['UDPSendPort'],pinling_config['UDPSendType']))



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

     # 创建一个响应对象
    # response = make_response("""
    # <!DOCTYPE html>
    # <html lang="en">
    #     <head>
    #         <meta charset="UTF-8">
    #         <meta http-equiv="X-UA-Compatible" content="IE=edge">
    #         <meta name="viewport" content="width=device-width, initial-scale=1.0">
    #         <title>提交成功</title>
    #         <script type="text/javascript">
    #             alert("修改成功");
    #             window.location.href = "/"; // 这里可以替换成提交表单后应该跳转的页面
    #         </script>
    #     </head>
    #     <body>
    #         如果您的浏览器没有弹出消息，请允许本站弹窗。
    #     </body>
    # </html>
    # """)

    # # 设置响应头部为HTML
    # response.headers["Content-Type"] = "text/html; charset=utf-8"
    # return response
    
    # return "camera_ip: {},gateway: {},netmask: {},udp_ip: {}".format(camera_ip,gateway,netmask,udp_ip)

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
