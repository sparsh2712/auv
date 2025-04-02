water_removal_constants = {
    "hue_mask_constants" : {
        "negative_mask_coverage": 12,
        "positive_mask_coverage" : 12, 
        "saturation_range" : (0, 255),
        "value_range": (100,255),
    },
    "saturation_mask_constants" :  {
        "hue_range" : (85,105),
        "saturation_range": (0,140),
        "value_range": (0,255)
    }
}

color_const={
    'red':([0,100,100],[200,255,255]),
    'blue':([106,0,0],[130,255,255]),
    'buoy_red':([70,100,100],[150,255,255]),
    'bin_red': ([150,0,0],[200,255,255])
}

morning = True
min_area = 2000 #600 #400

eve_color_const={
    'red':([170,0,0],[200,255,255]),
    'blue':([106,0,0],[130,255,255]),
    'bin_red': ([170,0,0],[200,255,255]),
    'buoy_red': ([170,0,0],[200,255,255])
}
eve_min_area = 600 #400

