
test_map = 'map2'

MARKER_MATERIAL_DICT = {
    0: 'normal',
    1: 'sheer',
    2: 'translate',
    3: 'broken',
    4: 'fade' 
}

if test_map == 'map0':
    ACTOR_TYPE_DICT = {
        0: 'person_1',
        1: 'person_2',
        3: 'bird_1',
        # 2: 'deer',
        # 3: 'bird_1',
        # 3: 'car_1',
        # 4: 'car_2',
        -1: 'unknown'
    }

    ACTOR_TYPE_DICT_INV = {
        'person_1': 0,
        'person_2': 1,
        'bird_1': 3,
        'unknown': -1
        # 'deer_1': 2,
        # 'bird_1': 3,
    }
elif test_map == 'map1':
    ACTOR_TYPE_DICT = {
        0: 'person_1',
        1: 'person_2',
        3: 'bird_1',
        # 2: 'deer',
        3: 'bird_1',
        4: 'dog_1',
        5: 'zebra_1',
        # 3: 'car_1',
        # 4: 'car_2',
        -1: 'unknown'
    }

    ACTOR_TYPE_DICT_INV = {
        'person_1': 0,
        'person_2': 1,
        'bird_1': 3,
        'dog_1': 4,
        'zebra_1': 5,
        'unknown': -1
        # 'deer_1': 2,
        # 'bird_1': 3,
    }
else:
    ACTOR_TYPE_DICT = {
    0: 'person_1',
    1: 'person_2',
    3: 'bird_1',
    4: 'dog_1',
    # 2: 'deer',
    # 3: 'bird_1',
    # 3: 'car_1',
    # 4: 'car_2',
    -1: 'unknown'
    }

    ACTOR_TYPE_DICT_INV = {
        'person_1': 0,
        'person_2': 1,
        'bird_1': 3,
        'dog_1': 4,
        'unknown': -1
        # 'deer_1': 2,
        # 'bird_1': 3,
    }

OBJ_DATABASE = {
    'person_1': 'ManBP',
    'person_2': 'ManBP_2',
    'deer': 'DeerBP',
    'car_1': 'npc_vehicle',
    'car_2': 'BP_SUV',
    'bird': 'CrowBlueprint',
}