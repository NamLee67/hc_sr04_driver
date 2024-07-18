![image](https://github.com/user-attachments/assets/b1d4c922-276a-4328-af86-c1e31c050098)
![image](https://github.com/user-attachments/assets/737d1eaa-ffdf-4158-a22d-7345bb68c83a)
Driver kernel linux
	Đọc dữ liệu khoảng cách thu được từ HC-SR04
•	int fd = open (struct inode *inode, struct file *filp);
-	inode : Thông tin của thiết bị đã được đăng kí
-	flilp : Chứa thông tin về thiết bị được mở, các cờ trạng thái
-	Mở thành công : Trả về giá trị 0
•	VD:  int fd = open(DEV_PATH, O_RDONLY);
-	DEV_PATH : Đường dẫn tới thiết bị đã đăng kí
-	O_RDONLY) : Cờ chỉ trạng thái mở tệp, trạng thái chỉ đọc
•	Ssize_t read(struct file *filp, char--user *buf, size-t count, loff-t *f_pos)
-	flilp : Chứa thông tin về thiết bị được mở, các cờ trạng thái
-	buf : Bộ buffer dùng để lưu trữ dữ liệu gửi lên user space 
-	count : Byte tối đa mà người dùng có thể đọc từ thiết bị
-	f_pos : Theo dõi vị trí hiện tại trong tệp, giúp quản lý việc đọc dữ liệu liên tiếp
•	VD:  ssize_t bytes-read = read(fd, &data, sizeof(data));
-	fd : Thiết bị được đăng kí muốn gọi ra
-	&data : trỏ tới bộ buffer uint32_t data;
-	sizeof(data) : số byte có thể đọc lấy theo kích thước của biến data là 32bit có thể lưu trữ từ 0 đến 2^32-1
•	hc_sr04_release(struct inode *inode, struct file *filp);
-	inode : Thông tin của thiết bị đã được đăng kí
-	flilp : Chứa thông tin về thiết bị được mở, các cờ trạng thái
•	VD: close(fd);
-	fd : Thiết bị được đăng kí muốn gọi ra
