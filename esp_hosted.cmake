# 1. 设置 ESP_HOSTED 根目录
set(ESP_HOSTED_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/esp-hosted")

# 2. 定义 ESP_HOSTED 源文件
# 按模块分组，提高可读性和可维护性
set(ESP_HOSTED_SRCS
        # 传输层
        "${ESP_HOSTED_ROOT}/drivers/transport/transport.c"

        # 内存池
        "${ESP_HOSTED_ROOT}/drivers/mempool/mempool.c"

        # 工具类
        "${ESP_HOSTED_ROOT}/host/utils/common.c"
        "${ESP_HOSTED_ROOT}/host/utils/util.c"

        # 公共模块
        "${ESP_HOSTED_ROOT}/common/esp_wrapper.c"
        "${ESP_HOSTED_ROOT}/proto/esp_hosted_rpc.pb-c.c"
        "${ESP_HOSTED_ROOT}/common/protobuf-c/protobuf-c/protobuf-c.c"

        # 主机初始化
        "${ESP_HOSTED_ROOT}/host/esp_hosted_host_init.c"

        # API 接口
        "${ESP_HOSTED_ROOT}/host/api/src/esp_wifi_weak.c"
        "${ESP_HOSTED_ROOT}/host/api/src/esp_hosted_api.c"

        # RPC 模块
        "${ESP_HOSTED_ROOT}/drivers/rpc/wrap/rpc_wrap.c"
        "${ESP_HOSTED_ROOT}/drivers/rpc/slaveif/rpc_slave_if.c"
        "${ESP_HOSTED_ROOT}/drivers/rpc/core/rpc_core.c"
        "${ESP_HOSTED_ROOT}/drivers/rpc/core/rpc_evt.c"
        "${ESP_HOSTED_ROOT}/drivers/rpc/core/rpc_req.c"
        "${ESP_HOSTED_ROOT}/drivers/rpc/core/rpc_rsp.c"

        # 串口驱动
        "${ESP_HOSTED_ROOT}/drivers/virtual_serial_if/serial_if.c"
        "${ESP_HOSTED_ROOT}/drivers/serial/serial_drv.c"
        "${ESP_HOSTED_ROOT}/drivers/serial/serial_ll_if.c"
)

# 3. 定义 ESP_HOSTED 包含目录
# 按模块分组，避免重复
list(APPEND ESP_HOSTED_ADD_INCLUDEDIRS
        # 驱动层
        "${ESP_HOSTED_ROOT}/drivers/transport"
        "${ESP_HOSTED_ROOT}/drivers/mempool"
        "${ESP_HOSTED_ROOT}/drivers/rpc"
        "${ESP_HOSTED_ROOT}/drivers/virtual_serial_if"
        "${ESP_HOSTED_ROOT}/drivers/serial"

        # 工具类
        "${ESP_HOSTED_ROOT}/host/utils"

        # API 接口
        "${ESP_HOSTED_ROOT}/host/api/include"

        # 公共模块
        "${ESP_HOSTED_ROOT}/common/include"
        "${ESP_HOSTED_ROOT}/proto"

        # 根目录
        "${ESP_HOSTED_ROOT}"
)

# 4. Protobuf 依赖
list(APPEND ESP_HOSTED_ADD_INCLUDEDIRS
        "${ESP_HOSTED_ROOT}/../protobuf-c"
)
