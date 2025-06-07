from parser import GrappleMapLoader, NAMED_LIMBS


loader: GrappleMapLoader = GrappleMapLoader.from_file("grapplemap_processed.json")

position_list = list(GrappleMapLoader.positions.values())

