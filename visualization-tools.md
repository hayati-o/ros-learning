# ROS Simülasyon ve Görselleştirme Araçları

ROS'ta sistemi anlamak, hata ayıklamak (debug) ve test etmek için kullanılan başlıca dört araç vardır. Bunların görevleri birbirinden tamamen farklıdır:

## 1.  Gazebo
* **Türü:** 3D Simülatör (Fizik Motorlu)
* **Görevi:** Robotu sanal bir dünyada *çalıştırır*. Yer çekimi, sürtünme, çarpışma algılama ve sensör verisi (Lidar, kamera) *üretme* işini yapar. 

## 2.  RViz (ROS Visualization)
* **Türü:** 3D Görselleştirici (Fizik Motoru Yok)
* **Görevi:** ROS topic'lerindeki *mevcut* veriyi görselleştirir. Robotun "ne gördüğünü" (örn. Lidar taraması, kamera görüntüsü) veya "ne planladığını" (örn. navigasyon rotası) gösterir.

## 3.  rqt_plot
* **Türü:** 2D Veri Çizdirici (Grafik)
* **Görevi:** Belirli bir ROS topic'indeki sayısal verilerin (örn. `/cmd_vel` hız komutu, sensörün sıcaklık verisi) zaman içindeki değişimini **grafik olarak** çizer. 

## 4.  rqt_graph
* **Türü:** Sistem Mimarisi Görselleştiricisi
* **Görevi:** O an çalışan tüm ROS **Nodes** (Düğümler) ve **Topics** (Konular) arasındaki bağlantıları bir "örümcek ağı" şeması olarak gösterir.

---

##  Kilit Farklar

> **Önemli Kural:** **Gazebo** veriyi *üretir* (simüle eder).
> 
> **RViz, rqt_plot ve rqt_graph** ise *mevcut* veriyi ve sistemi *gösterir* (görselleştirir ve hata ayıklamaya yarar).

##  Terminal Komutları

Bu araçları başlatmak için kullanılan temel komutlar:

```bash
# Gazebo'yu başlat
rosrun gazebo_ros gazebo

# RViz'i başlat
rosrun rviz rviz

# rqt_plot'u başlat
rosrun rqt_plot rqt_plot

# rqt_graph'ı başlat
rosrun rqt_graph rqt_graph
