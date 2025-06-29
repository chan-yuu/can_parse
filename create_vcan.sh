echo -e "${YELLOW}================ 开始创建虚拟CAN设备 ================${NC}"

# 安装can-utils（确保必备工具）
echo -e "${YELLOW}正在安装can-utils工具...${NC}"
sudo apt-get install -y can-utils
if [ $? -ne 0 ]; then
    echo -e "${RED}can-utils安装失败，请检查网络或手动安装${NC}"
    return
fi
echo -e "${GREEN}can-utils安装成功${NC}"

# 加载vcan内核模块
echo -e "${YELLOW}正在加载vcan内核模块...${NC}"
sudo modprobe vcan
if [ $? -ne 0 ]; then
    echo -e "${RED}vcan模块加载失败，请检查内核支持${NC}"
    return
fi
echo -e "${GREEN}vcan模块加载成功${NC}"

# 获取用户输入的设备名称
read -p "请输入要创建的CAN设备名称 (例如: can0): " can_device

# 检查设备是否已存在
if ip link show $can_device >/dev/null 2>&1; then
    echo -e "${YELLOW}设备 $can_device 已存在，是否启动它? (y/n): ${NC}"
    read -p "" choice
    if [[ $choice == "y" || $choice == "Y" ]]; then
        sudo ip link set $can_device up
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}设备 $can_device 已启动${NC}"
        else
            echo -e "${RED}启动设备失败${NC}"
        fi
        return
    else
        echo -e "${YELLOW}已取消操作${NC}"
        return
    fi
fi

# 创建虚拟CAN设备
echo -e "${YELLOW}正在创建设备 $can_device...${NC}"
sudo ip link add dev $can_device type vcan
if [ $? -eq 0 ]; then
    # 启动设备
    sudo ip link set $can_device up
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}虚拟CAN设备 $can_device 创建并启动成功${NC}"
        echo -e "${YELLOW}测试命令:${NC}"
        echo -e "  ${CYAN}cansend $can_device 123#11223344${NC}"
        echo -e "  ${CYAN}candump $can_device${NC}"
    else
        echo -e "${YELLOW}设备创建成功但启动失败，已创建未启动的设备${NC}"
        echo -e "${YELLOW}启动命令:${NC} ${CYAN}sudo ip link set $can_device up${NC}"
    fi
else
    echo -e "${RED}创建设备 $can_device 失败，请尝试其他名称${NC}"
fi

echo -e "${YELLOW}================ 虚拟CAN设备创建完成 ================${NC}"
