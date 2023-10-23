#ifndef _ITRACKER_H_
#define _ITRACKER_H_

class itracker
{
public:
    itracker();
    ~itracker();

    void init(const cv::Rect &roi, cv::Mat image);
    void update(cv::Mat image);
    void reset();

private:
    bool m_init;
};


#endif