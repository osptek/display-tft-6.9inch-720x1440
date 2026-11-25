idf_component_register(SRCS "esp32p4-6.9in-720x1440-st7121p-mipi_bringup.c" "st7121p_lcd.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_lcd esp_hw_support esp_driver_gpio esp_driver_i2c)
