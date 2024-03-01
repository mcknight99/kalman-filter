import csv
for i in range(20):
    total_list = []
    final_list = []
    filename_input = str(i + 1) + '_parsed.csv'
    filename_output = str(i + 1) + '_standardized.csv'
    converstion_meter = 0.3048

    with open(filename_input, 'r') as csv_file:
        reader = csv.reader(csv_file)

        for row in reader:
            total_list.append(row)

    total_list = total_list[1:]


    time = []
    baro_altitude = []
    accl_x = []
    accl_y = []
    accl_z = []
    gps_altitude = []
    gyro_roll = []
    gyro_pitch = []
    gyro_yaw = []

    for i in range(len(total_list)):
        time.append(total_list[i][0])
        baro_altitude.append(total_list[i][1])
        accl_x.append(total_list[i][2])
        accl_y.append(total_list[i][3])
        accl_z.append(total_list[i][4])
        gps_altitude.append(total_list[i][5])
        gyro_roll.append(total_list[i][6])
        gyro_pitch.append(total_list[i][7])
        gyro_yaw.append(total_list[i][8])

    time = [float(i) for i in time]
    baro_altitude = [float(i) for i in baro_altitude]
    accl_x = [float(i) for i in accl_x]
    accl_y = [float(i) for i in accl_y]
    accl_z = [float(i) for i in accl_z]
    gps_altitude = [float(i) for i in gps_altitude]
    gyro_roll = [float(i) for i in gyro_roll]
    gyro_pitch = [float(i) for i in gyro_pitch]
    gyro_yaw = [float(i) for i in gyro_yaw]

    init_time = time[0]
    init_baro_alt = baro_altitude[0]
    init_gps_alt = gps_altitude[0]

    time = [(i - init_time) for i in time]
    baro_altitude = [(i - init_baro_alt) * converstion_meter for i in baro_altitude]
    gps_altitude = [(i - init_gps_alt) * converstion_meter for i in gps_altitude]


    accl_x = [(i  * converstion_meter) for i in accl_x]
    accl_y = [(i  * converstion_meter) for i in accl_y]
    accl_z = [(i  * converstion_meter) for i in accl_z]

    for i in range(len(time)):
        temp_list = []
        temp_list.append(time[i])
        temp_list.append(baro_altitude[i])
        temp_list.append(accl_x[i])
        temp_list.append(accl_y[i])
        temp_list.append(accl_z[i])
        temp_list.append(gps_altitude[i])
        temp_list.append(gyro_roll[i])
        temp_list.append(gyro_pitch[i])
        temp_list.append(gyro_yaw[i])
        final_list.append(temp_list)

    header  = ['time', 'baro_altitude', 'accl_x', 'accl_y', 'accl_z', 'gps_altitude', 'gyro_roll', 'gyro_pitch', 'gyro_yaw'] 

    with open(filename_output, 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)

        # write the header
        writer.writerow(header)

        # write the data
        writer.writerows(final_list)

