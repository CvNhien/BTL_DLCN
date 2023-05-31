# BTL_DLCN

Sơ đồ khối kết nối điện giữa mạch điều khiển PT100 , vi điều khiển STM32F407VG và giao diện hiển thị

![image](https://github.com/CvNhien/BTL_DLCN/assets/111190445/5af51987-7b07-4ae1-b481-4bc7ce384d07)

Khi đầu ra của mạch điều khiển PT100 được đọc bởi cổng ADC1 (PA2) của chip STM32F4, mô-đun chuyển đổi tương tự sang số ADC sẽ khởi động và giá trị điện áp được đọc tại Cổng ADC1 (PA2) sẽ được sử dụng giá trị AD 12 bit tại thời điểm này, giá trị AD tính toán nhiệt độ tương ứng của AD thông qua công thức tính toán và hiển thị nhiệt độ với nhau. Khi được chuyển đổi nhiệt độ đầu ra đạt đến yêu cầu báo động, đèn led trên board sẽ sáng.

Lưu đồ chương trình tổng thể.

![image](https://github.com/CvNhien/BTL_DLCN/assets/111190445/2595a731-26bc-4f74-b8bb-47e46da3cf3b)

Firmware: STM32F4 trên KeliC IDE, sử dụng thư viện STD
  + Calib PT100 tầm do 0 độ C đến 100 độ C (điện áp ngõ ra tương ứng 0V - 3v)
  + ADC + DMA
  + Timer
  + UART + DMA
 
 Software: sử dụng phần mềm Qt creator 5.12 (C++)
 
 ![image](https://github.com/CvNhien/BTL_DLCN/assets/111190445/bbdf2594-c604-4b61-911a-c6ea532f4eda)

•	Nút Start để bắt đầu hiển thị nhiệt độ được đo

•	Nút Stop để dừng hiển thị nhiệt độ

•	Enable Time cho phép nhập thời gian tùy chọn

•	Enable Alarm để bật hiển thị cảnh báo
