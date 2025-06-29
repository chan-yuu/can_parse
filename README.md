# 安装库
pip install python-can cantools

sudo apt install can-utils

# python
python3 send_can.py

python parse_can.py

# cpp
g++ -o can_pub can_pub.cpp

./can_pub ./Yokee_CAN2_VCU_500k.dbc


g++ -o can_parser_v1 can_parse_v1.cpp

./can_parser_v1 ./Yokee_CAN2_VCU_500k.dbc
