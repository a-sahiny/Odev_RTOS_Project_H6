# MP257 Cortex-M33 Zephyr uygulamasının derleme tanımı.

# Zephyr 4.5.0'ın istediği en düşük CMake sürümü
cmake_minimum_required(VERSION 3.28)

# project()'ten önce gelmeli: kartı ve derleyiciyi bu satır ayarlar, app hedefini de oluşturur
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(mp257_m33 LANGUAGES C)

# Derlenecek kaynaklar tek tek listelenir
target_sources(app PRIVATE src/main.c src/app_isr.c src/packet.c src/ipcc.c src/rpmsg_link.c)

# RPMsg kütüphanesinin başlıkları
target_include_directories(app PRIVATE ${LIBMETAL_INCLUDE_DIR} ${OPENAMP_INCLUDE_DIR})
