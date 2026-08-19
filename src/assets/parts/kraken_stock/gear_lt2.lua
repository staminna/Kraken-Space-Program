part {
  id = "kraken.gear.lt2",
  author = "Claude",
  shape = { kind = "landing_leg", radius = 1.9 },
  display_name = "LT-2 Landing Strut Assembly",
  manufacturer = "Seraphina Aerospace Industries",
  description = "Four legs and a hub. SAI's marketing insists the correct term is 'terminal descent contact system', but everyone in the assembly building calls them the legs, including the people who write the manuals.",
  categories = {"structural", "landing", "1.25m"},
  mass = 0.2,  -- tonnes, dry (the loader converts to kg)
  drag_coefficient = 0.5,  -- splayed struts are not aerodynamic
  attach_nodes = {
    top = {
      position         = vec3(0, 0.3, 0),
      size             = 2,  -- 1.25 meter
      tensile_strength = 480.0,  -- kN before joint breaks under pull
      shear_strength   = 360.0,  -- kN before joint breaks under shear
    },
  },
}
