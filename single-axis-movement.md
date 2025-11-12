# Uygulama: Tek Eksen Boyunca Hareket (Python & Gazebo)

Bu uygulamada, bir TurtleBot3 robotunu Gazebo simülasyonunda başlatıp, yazdığımız Python node'u ile belirli bir mesafe (veya süre) boyunca tek bir eksende (düz) hareket ettireceğiz.

## 1. Ön Hazırlıklar (Paket ve Derleme)

Bu uygulama için `basit_uygulamalar` adında bir paket oluşturuyoruz. (`catkin_create_pkg basit_uygulamalar`)

1.  Paketi oluşturduktan veya değiştirdikten sonra derliyoruz:
    ```bash
    cd ..
    catkin_make
    ```
2.  Gerekli klasörleri paketimizin içine oluşturuyoruz:
    ```bash
    roscd basit_uygulamalar/
    mkdir script
    mkdir launch
    ```

## 2. Adım: Python Node'u (`duz_git_v1.py`)

`script` klasörünün içine robotumuzu hareket ettirecek Python kodumuzu ekliyoruz.

* **Dosya:** `~/catkin_ws/src/basit_uygulamalar/script/duz_git_v1.py`
* **İçerik:**

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Uygulama 1: Tek Eksen Boyunca Hareket I
"""

import rospy
from geometry_msgs.msg import Twist

def hareket():
    # 'duz_git' adında bir node başlat
    rospy.init_node('duz_git', anonymous=True)
    
    # 'cmd_vel' topic'ine Twist tipinde mesaj yayınlayacak bir publisher oluştur
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Hız mesajını hazırla
    hiz_mesaji = Twist()
    hiz_mesaji.linear.x = 0.5  # X ekseninde 0.5 m/s hızla ileri
    
    # Hareket parametreleri
    mesafe = 5           # 5 birimlik "hedef" (bu kodda zamanla çarpıldığı için birim: m)
    yer_degistirme = 0
    t0 = rospy.Time.now().to_sec() # Başlangıç zamanı

    # Hedeflenen yer değiştirmeye ulaşana kadar hız mesajını yayınla
    while (yer_degistirme < mesafe):
        # Hız mesajını yayınla
        pub.publish(hiz_mesaji)
        
        # Geçen süreyi hesapla
        ti = rospy.Time.now().to_sec()
        
        # Yer değiştirme = hız * (geçen süre)
        yer_degistirme = hiz_mesaji.linear.x * (ti - t0)
        
    # Döngü bittiğinde (hedefe varıldığında) robotu durdur
    hiz_mesaji.linear.x = 0.0
    pub.publish(hiz_mesaji)
    rospy.loginfo("Hedefe varildi !")


hareket()
```

## 3. Adım: Python Script'i Çalıştırılabilir Yapma

ROS'un bu dosyayı bir program olarak çalıştırabilmesi için ona "çalıştırma izni" vermemiz gerekiyor.
```Bash

# Script klasörüne git
cd ~/catkin_ws/src/basit_uygulamalar/script

# Dosyaya çalıştırma izni ver
chmod +x duz_git_v1.py
```
## 4. Adım: Simülasyon Ortamı (bos_dunya.launch)

Robotu (TurtleBot3) boş bir Gazebo dünyasında başlatmak için bir .launch dosyasına ihtiyacımız var.

Mevcut turtlebot3_gazebo paketindeki turtlebot3_empty_world.launch dosyasını, kendi paketimizin launch klasörüne bos_dunya.launch adıyla kopyalıyoruz.
```Bash

# turtlebot3_simulations paketinin konumunu bul (örnek yol)
# Gerçek yolu 'rospack find turtlebot3_gazebo' ile bulabilirsiniz
cd ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch

# Launch dosyasını kendi paketimize kopyala
cp turtlebot3_empty_world.launch ~/catkin_ws/src/basit_uygulamalar/launch/bos_dunya.launch

Artık kendi paketimiz üzerinden Gazebo'yu başlatabiliriz.
```
## 5. Adım: Uygulamayı Çalıştırma

İki ayrı terminale ihtiyacımız olacak (roscore'un çalıştığını varsayıyoruz).

Terminal 1: Gazebo ve Robotu Başlat

Bu komut, bos_dunya.launch dosyasını çalıştırır, Gazebo'yu açar ve içine TurtleBot3 robotunu yükler.
```Bash

roslaunch basit_uygulamalar bos_dunya.launch
```

Terminal 2: Python Node'unu Çalıştır

Bu komut, bizim yazdığımız duz_git_v1.py script'ini çalıştırır.
```Bash

rosrun basit_uygulamalar duz_git_v1.py
```
## 6. Gözlem (Beklenen Sonuç)
 * roslaunch komutundan sonra Gazebo penceresi açılır ve boş bir dünyada TurtleBot3 robotu belirir.

 * rosrun komutunu çalıştırdığınız anda, Python node'u cmd_vel topic'ine linear.x = 0.5 olan hız mesajları yayınlamaya başlar.

 * Gazebo'daki robot ileri doğru hareket etmeye başlar.

 * Yaklaşık 10 saniye sonra (mesafe 5 / hız 0.5 = 10 saniye), Python script'i Hedefe varildi ! mesajını loglar ve robota durma komutu (hız = 0.0) gönderir.

 * Gazebo'daki robot durur.
