import pandas as pd

def load_identifiers(config_file_name='colors.csv'):
    identifiers = []
    env_config_data_frame = pd.read_csv(config_file_name, index_col=0)
    num_intersections = len(env_config_data_frame.values)
    for ii in range(num_intersections):
        config_line = env_config_data_frame.iloc[[ii]]
        color1 = config_line.values[0][0:3]
        prob1 = config_line.values[0][3]
        color2 = config_line.values[0][4:7]
        prob2 = config_line.values[0][7]
        color3 = config_line.values[0][8:11]
        prob3 = config_line.values[0][11]
        identifiers.append([(color1, prob1), (color2, prob2), (color3, prob3)])
    return identifiers

load_identifiers()
