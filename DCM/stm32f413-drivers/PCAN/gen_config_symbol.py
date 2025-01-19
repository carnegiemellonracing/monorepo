for drv in range(6):
    for board_idx, board in enumerate(["DIM", "CDC"]):
        for config in range(5):
            hex_id = hex(0x600 + drv*10 + board_idx*5 + config)[2:]
            val = f"""[{board}_Config{config}_Drv{drv}]
ID={hex_id}h
DLC=4
CycleTime=1000
Timeout=10000
Var={board}_config{config}_drv{drv}_config_val_1 unsigned 0,8
Var={board}_config{config}_drv{drv}_config_val_2 unsigned 8,8
Var={board}_config{config}_drv{drv}_config_val_3 unsigned 16,8
Var={board}_config{config}_drv{drv}_config_val_4 unsigned 24,8\n\n"""
            print(val)