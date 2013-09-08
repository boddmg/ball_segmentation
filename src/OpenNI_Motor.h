#include <XnUSB.h>
class KinectMotors
{
public:
    enum { MaxDevs = 16 };

public:
    KinectMotors();
    virtual ~KinectMotors();
    
    /**
        * Open device.
        * @return true if succeeded, false - overwise
        */
    bool Open();
    
    /**
        * Close device.
        */
    void Close();
    
    /**
        * Move motor up or down to specified angle value.
        * @param angle angle value
        * @return true if succeeded, false - overwise
        */
    bool Move(int angle);
    
private:
    XN_USB_DEV_HANDLE m_devs[MaxDevs];
    XnUInt32 m_num;
    bool m_isOpen;
};