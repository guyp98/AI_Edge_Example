/**
 * Copyright 2020 (C) Hailo Technologies Ltd.
 * All rights reserved.
 *
 * Hailo Technologies Ltd. ("Hailo") disclaims any warranties, including, but not limited to,
 * the implied warranties of merchantability and fitness for a particular purpose.
 * This software is provided on an "AS IS" basis, and Hailo has no obligation to provide maintenance,
 * support, updates, enhancements, or modifications.
 *
 * You may use this software in the development of any project.
 * You shall not reproduce, modify or distribute this software without prior written permission.
 **/
/**
 * @ file example_common.h
 * Common macros and defines used by Hailort Examples
 **/

#ifndef _EXAMPLE_COMMON_H_
#define _EXAMPLE_COMMON_H_

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include "hailo/hailort.h"

#include <opencv2/opencv.hpp>

#define INPUT_FILES_COUNT (10)


#define FREE(var)                           \
    do {                                    \
        if (NULL != (var)) {                \
            free(var);                      \
            var = NULL;                     \
        }                                   \
    } while(0)

#define REQUIRE_ACTION(cond, action, label, ...) \
    do {                                         \
        if (!(cond)) {                           \
            printf(__VA_ARGS__);                 \
            printf("\n");                        \
            action;                              \
            goto label;                          \
        }                                        \
    } while(0)

#define REQUIRE_SUCCESS(status, label, ...) REQUIRE_ACTION((HAILO_SUCCESS == (status)), , label, __VA_ARGS__)

#define ARRAY_LENGTH(__array) (sizeof((__array)) / sizeof((__array)[0]))

#define NSEC_IN_SEC (1e+9)

typedef struct {
    int num_images;
    size_t input_stream_cnt;
    size_t output_stream_cnt;
    struct timespec start_time;
    struct timespec end_time;
    hailo_stream_info_t* input_vstream_infos;
    hailo_stream_info_t* output_vstream_infos;
    hailo_input_vstream_params_by_name_t* input_vstream_params;
    hailo_output_vstream_params_by_name_t* output_vstream_params;
} statistics;

typedef struct {
    hailo_input_vstream input_vstream;
    int num_images;
    size_t tid;
} thread_send_args;

typedef struct {
    hailo_output_vstream output_vstream;
    int num_images;
    size_t tid;
} thread_recv_args;

const char *get_transform_string(hailo_stream_transform_mode_t transform)
{
    switch(transform) {
        case HAILO_STREAM_NO_TRANSFORM: return "NO";
        case HAILO_STREAM_TRANSFORM_COPY: return "COPY";
        case HAILO_STREAM_MAX_ENUM:
        default: return "Wrong";
    }
}

const char *get_flags_string(int flags)
{
    switch(flags) {
        case 0: return "NONE";
        case 1: return "QUANT";
        case 2: return "TRANS";
        case 3: return "QT+TR";
        default: return "UNKNOWN";
    }
}

const char *get_type_string(int type)
{
    switch(type) {
        case 0: return "AUTO";
        case 1: return "UINT8";
        case 2: return "UINT16";
        case 3: return "FLOAT32";
        default: return "UNKNOWN";
    }
}

const char *get_order_string(int order)
{
    switch(order) {
        case 0: return "NHWC";
        case 1: return "NHCW";
        case 2: return "FCR";
        case 3: return "F8CR";
        case 5: return "NC";
        case 8: return "NMS";
        case 10: return "NCHW";
        default: return "UNKNOWN";
    }
}



#pragma once

#include <opencv2/opencv.hpp>

// Transformations were taken from https://stackoverflow.com/questions/17892346/how-to-convert-rgb-yuv-rgb-both-ways.
#define RGB2Y(R, G, B) CLIP((0.257 * (R) + 0.504 * (G) + 0.098 * (B)) + 16)
#define RGB2U(R, G, B) CLIP((-0.148 * (R)-0.291 * (G) + 0.439 * (B)) + 128)
#define RGB2V(R, G, B) CLIP((0.439 * (R)-0.368 * (G)-0.071 * (B)) + 128)

typedef enum
{
    HAILO_MAT_RGB,
    HAILO_MAT_RGBA,
    HAILO_MAT_YUY2,
    HAILO_MAT_NV12
} hailo_mat_t;

typedef enum
{
    NONE = -1,
    VERTICAL,
    HORIZONTAL,
    DIAGONAL,
    ANTI_DIAGONAL,
} LineOrientation;

inline LineOrientation line_orientation(cv::Point point1, cv::Point point2)
{
    if (point1.x == point2.x)
        return VERTICAL;
    else if (point1.y == point2.y)
        return HORIZONTAL;
    else if (point1.x < point2.x && point1.y < point2.y)
        return DIAGONAL;
    else if (point1.x < point2.x && point1.y > point2.y)
        return ANTI_DIAGONAL;
    else
        return NONE;
}

inline int floor_to_even_number(int x)
{
    /*
    The expression x &~1 in C++ performs a bitwise AND operation between the number x and the number ~1(bitwise negation of 1).
    In binary representation, the number 1 is represented as 0000 0001, and its negation, ~1, is equal to 1111 1110.
    The bitwise AND operation between x and ~1 zeros out the least significant bit of x,
    effectively rounding it down to the nearest even number.
    This is because any odd number in binary representation will have its least significant bit set to 1,
    and ANDing it with ~1 will zero out that bit.
    */
    return x & ~1;
}

class HailoMat
{
protected:
    uint m_height;
    uint m_width;
    uint m_native_height;
    uint m_native_width;
    uint m_stride;
    int m_line_thickness;
    int m_font_thickness;

public:
    HailoMat(uint height, uint width, uint stride, int line_thickness = 1, int font_thickness = 1) : m_height(height),
                                                                                                     m_width(width),
                                                                                                     m_native_height(height),
                                                                                                     m_native_width(width),
                                                                                                     m_stride(stride),
                                                                                                     m_line_thickness(line_thickness),
                                                                                                     m_font_thickness(font_thickness){};
    HailoMat() : m_height(0), m_width(0), m_native_height(0), m_native_width(0), m_stride(0), m_line_thickness(0), m_font_thickness(0){};
    virtual ~HailoMat() = default;
    uint width() { return m_width; };
    uint height() { return m_height; };
    uint native_width() { return m_native_width; };
    uint native_height() { return m_native_height; };
    virtual cv::Mat &get_mat() = 0;
    /*
     * @brief Crop ROIs from the mat, note the present implementation is valid
     *        for interlaced formats. Planar formats such as NV12 should override.
     *
     * @param crop_roi
     *        The roi to crop from this mat.
     * @return cv::Mat
     *         The cropped mat.
     */
    /**
     * @brief Get the type of mat
     *
     * @return hailo_mat_t - The type of the mat.
     */
    virtual hailo_mat_t get_type() = 0;
};

class HailoRGBMat : public HailoMat
{
protected:
    cv::Mat m_mat;
    std::string m_name;

public:
    HailoRGBMat(uint8_t *buffer, uint height, uint width, uint stride, int line_thickness = 1, int font_thickness = 1, std::string name = "HailoRGBMat") : HailoMat(height, width, stride, line_thickness, font_thickness)
    {
        m_name = name;
        m_mat = cv::Mat(m_height, m_width, CV_8UC3, buffer, m_stride);
    };
    HailoRGBMat(cv::Mat mat, std::string name, int line_thickness = 1, int font_thickness = 1)
    {
        m_mat = mat;
        m_name = name;
        m_height = mat.rows;
        m_width = mat.cols;
        m_stride = mat.step;
        m_native_height = m_height;
        m_native_width = m_width;
        m_line_thickness = line_thickness;
        m_font_thickness = font_thickness;
    }
    virtual ~HailoRGBMat()
    {
        m_mat.release();
    }
};



#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

#endif /* _EXAMPLE_COMMON_H_ */
