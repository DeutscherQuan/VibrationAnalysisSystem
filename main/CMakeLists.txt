set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "wifi.c" "main.c" "i2c.c" "adxl345.c")
set(COMPONENT_EMBED_TXTFILES "microprocessor.pem" "vip1.pem.crt" "private.pem.key")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()