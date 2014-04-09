#ifndef DVSCALIBRATION_H_
#define DVSCALIBRATION_H_

class DVSCalibration
{
public:
  DVSCalibration(int dots);

  bool add_image();
  int get_num_good_images();
  bool calibrate();
  bool save_calibration();

private:
  int dots;
};

#endif /* DVSCALIBRATION_H_ */
