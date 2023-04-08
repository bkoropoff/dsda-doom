//
// Copyright(C) 2021 by Ryan Krafnick
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// DESCRIPTION:
//	DSDA Map Format
//

#include "doomstat.h"
#include "lprintf.h"
#include "p_spec.h"
#include "r_state.h"
#include "w_wad.h"
#include "p_map.h"
#include "p_setup.h"
#include "p_user.h"
#include "hexen/p_anim.h"

#include "dsda/args.h"
#include "dsda/thing_id.h"

#include "map_format.h"

map_format_t map_format;

typedef enum {
  door_type_none = -1,
  door_type_red,
  door_type_blue,
  door_type_yellow,
  door_type_unknown = door_type_yellow,
  door_type_multiple
} door_type_t;

int dsda_DoorType(int index) {
  int special = lines[index].special;

  if (map_format.hexen) {
    if (special == 13 || special == 83)
      return door_type_unknown;

    return door_type_none;
  }

  if (heretic && special > 34)
    return door_type_none;

  if (GenLockedBase <= special && special < GenDoorBase) {
    special -= GenLockedBase;
    special = (special & LockedKey) >> LockedKeyShift;
    if (!special || special == 7)
      return door_type_multiple;
    else
      return (special - 1) % 3;
  }

  switch (special) {
    case 26:
    case 32:
    case 99:
    case 133:
      return door_type_blue;
    case 27:
    case 34:
    case 136:
    case 137:
      return door_type_yellow;
    case 28:
    case 33:
    case 134:
    case 135:
      return door_type_red;
    default:
      return door_type_none;
  }
}

dboolean dsda_IsExitLine(int index) {
  int special = lines[index].special;

  if (map_format.hexen)
    return special == 74 ||
           special == 75;

  return special == 11  ||
         special == 52  ||
         special == 197 ||
         special == 51  ||
         special == 124 ||
         special == 198;
}

dboolean dsda_IsTeleportLine(int index) {
  int special = lines[index].special;

  if (map_format.hexen)
    return special == 70 ||
           special == 71;

  if (heretic)
    return special == 39;

  return special == 39  ||
         special == 97  ||
         special == 125 ||
         special == 126;
}

// Migrate some non-hexen data to hexen format
static void dsda_MigrateMobjInfo(void) {
  int i;
  static dboolean migrated = false;

  if (hexen) return;

  if (map_format.zdoom && !migrated)
  {
    migrated = true;

    for (i = mobj_types_zero; i < num_mobj_types; ++i) {
      if (mobjinfo[i].flags & MF_COUNTKILL)
        mobjinfo[i].flags2 |= MF2_MCROSS | MF2_PUSHWALL | MF2_CANUSEWALLS;

      if (mobjinfo[i].flags & MF_MISSILE)
        mobjinfo[i].flags2 |= MF2_PCROSS | MF2_IMPACT;
    }

    if (!raven) {
      mobjinfo[MT_SKULL].flags2 |= MF2_MCROSS | MF2_PUSHWALL | MF2_CANUSEWALLS;
      mobjinfo[MT_PLAYER].flags2 |= MF2_WINDTHRUST | MF2_PUSHWALL | MF2_CANUSEWALLS;
    }
  }
  else if (!map_format.zdoom && migrated)
  {
    migrated = false;

    for (i = mobj_types_zero; i < num_mobj_types; ++i) {
      if (mobjinfo[i].flags & MF_COUNTKILL)
        mobjinfo[i].flags2 &= ~(MF2_MCROSS | MF2_PUSHWALL | MF2_CANUSEWALLS);

      if (mobjinfo[i].flags & MF_MISSILE)
        mobjinfo[i].flags2 &= ~(MF2_PCROSS | MF2_IMPACT);
    }

    if (!raven) {
      mobjinfo[MT_SKULL].flags2 &= ~(MF2_MCROSS | MF2_PUSHWALL | MF2_CANUSEWALLS);
      mobjinfo[MT_PLAYER].flags2 &= ~(MF2_WINDTHRUST | MF2_PUSHWALL | MF2_CANUSEWALLS);
    }
  }
}

static const map_format_t zdoom_in_hexen_map_format = {
  .zdoom = true,
  .hexen = true,
  .polyobjs = true,
  .acs = false,
  .thing_id = true,
  .mapinfo = false,
  .sndseq = false,
  .sndinfo = false,
  .animdefs = false,
  .doublesky = false,
  .map99 = false,
  .lax_monster_activation = true,
  .generalized_mask = ~0xff,
  .switch_activation = SPAC_USE | SPAC_IMPACT | SPAC_PUSH,
  .init_sector_special = P_SpawnZDoomSectorSpecial,
  .player_in_special_sector = P_PlayerInZDoomSector,
  .mobj_in_special_sector = P_MobjInZDoomSector,
  .spawn_scroller = P_SpawnZDoomScroller,
  .spawn_friction = P_SpawnZDoomFriction,
  .spawn_pusher = P_SpawnZDoomPusher,
  .spawn_extra = P_SpawnZDoomExtra,
  .cross_special_line = P_CrossZDoomSpecialLine,
  .shoot_special_line = P_ShootHexenSpecialLine,
  .test_activate_line = P_TestActivateZDoomLine,
  .execute_line_special = P_ExecuteZDoomLineSpecial,
  .post_process_line_special = P_PostProcessZDoomLineSpecial,
  .post_process_sidedef_special = P_PostProcessZDoomSidedefSpecial,
  .animate_surfaces = P_AnimateZDoomSurfaces,
  .check_impact = P_CheckZDoomImpact,
  .translate_line_flags = P_TranslateZDoomLineFlags,
  .apply_sector_movement_special = P_ApplyHereticSectorMovementSpecial,
  .t_vertical_door = T_VerticalCompatibleDoor,
  .t_move_floor = T_MoveCompatibleFloor,
  .t_move_ceiling = T_MoveCompatibleCeiling,
  .t_build_pillar = T_BuildZDoomPillar,
  .t_plat_raise = T_ZDoomPlatRaise,
  .ev_teleport = EV_CompatibleTeleport,
  .player_thrust = P_CompatiblePlayerThrust,
  .build_mobj_thing_id_list = dsda_BuildMobjThingIDList,
  .add_mobj_thing_id = dsda_AddMobjThingID,
  .remove_mobj_thing_id = dsda_RemoveMobjThingID,
  .iterate_spechit = P_IterateZDoomSpecHit,
  .mapthing_size = sizeof(hexen_mapthing_t),
  .maplinedef_size = sizeof(hexen_maplinedef_t),
  .mt_push = MT_PUSH,
  .mt_pull = MT_PULL,
  .dn_polyanchor = 9300,
  .dn_polyspawn_start = 9301,
  .dn_polyspawn_hurt = 9303,
  .dn_polyspawn_end = 9303,
  .visibility = VF_ZDOOM | VF_DOOM,
};

static const map_format_t hexen_map_format = {
  .zdoom = false,
  .hexen = true,
  .polyobjs = true,
  .acs = true,
  .thing_id = true,
  .mapinfo = true,
  .sndseq = true,
  .sndinfo = true,
  .animdefs = true,
  .doublesky = true,
  .map99 = true,
  .lax_monster_activation = false,
  .generalized_mask = 0, // not used
  .switch_activation = SPAC_USE | SPAC_IMPACT,
  .init_sector_special = NULL, // not used
  .player_in_special_sector = P_PlayerInHexenSector,
  .mobj_in_special_sector = P_MobjInHexenSector,
  .spawn_scroller = NULL, // not used
  .spawn_friction = NULL, // not used
  .spawn_pusher = NULL, // not used
  .spawn_extra = NULL, // not used
  .cross_special_line = P_CrossHexenSpecialLine,
  .shoot_special_line = P_ShootHexenSpecialLine,
  .test_activate_line = P_TestActivateHexenLine,
  .execute_line_special = P_ExecuteHexenLineSpecial,
  .post_process_line_special = P_PostProcessHexenLineSpecial,
  .post_process_sidedef_special = P_PostProcessHexenSidedefSpecial,
  .animate_surfaces = P_AnimateHexenSurfaces,
  .check_impact = NULL, // not used
  .translate_line_flags = P_TranslateHexenLineFlags,
  .apply_sector_movement_special = P_ApplyHereticSectorMovementSpecial,
  .t_vertical_door = T_VerticalHexenDoor,
  .t_move_floor = T_MoveHexenFloor,
  .t_move_ceiling = T_MoveHexenCeiling,
  .t_build_pillar = T_BuildHexenPillar,
  .t_plat_raise = T_HexenPlatRaise,
  .ev_teleport = NULL, // not used
  .player_thrust = P_HexenPlayerThrust,
  .build_mobj_thing_id_list = P_CreateTIDList,
  .add_mobj_thing_id = P_InsertMobjIntoTIDList,
  .remove_mobj_thing_id = P_RemoveMobjFromTIDList,
  .iterate_spechit = NULL, // not used
  .mapthing_size = sizeof(hexen_mapthing_t),
  .maplinedef_size = sizeof(hexen_maplinedef_t),
  .mt_push = -1,
  .mt_pull = -1,
  .dn_polyanchor = 3000,
  .dn_polyspawn_start = 3001,
  .dn_polyspawn_hurt = -1,
  .dn_polyspawn_end = 3002,
  .visibility = VF_HEXEN,
};

static const map_format_t heretic_map_format = {
  .zdoom = false,
  .hexen = false,
  .polyobjs = false,
  .acs = false,
  .thing_id = false,
  .mapinfo = false,
  .sndseq = false,
  .sndinfo = false,
  .animdefs = false,
  .doublesky = false,
  .map99 = false,
  .lax_monster_activation = true,
  .generalized_mask = 0,
  .switch_activation = 0, // not used
  .init_sector_special = P_SpawnCompatibleSectorSpecial,
  .player_in_special_sector = P_PlayerInHereticSector,
  .mobj_in_special_sector = P_MobjInHereticSector,
  .spawn_scroller = P_SpawnCompatibleScroller,
  .spawn_friction = P_SpawnCompatibleFriction,
  .spawn_pusher = P_SpawnCompatiblePusher,
  .spawn_extra = P_SpawnCompatibleExtra,
  .cross_special_line = P_CrossHereticSpecialLine,
  .shoot_special_line = P_ShootCompatibleSpecialLine,
  .test_activate_line = NULL, // not used
  .execute_line_special = NULL, // not used
  .post_process_line_special = P_PostProcessHereticLineSpecial,
  .post_process_sidedef_special = P_PostProcessHereticSidedefSpecial,
  .animate_surfaces = P_AnimateHereticSurfaces,
  .check_impact = P_CheckHereticImpact,
  .translate_line_flags = P_TranslateCompatibleLineFlags,
  .apply_sector_movement_special = P_ApplyHereticSectorMovementSpecial,
  .t_vertical_door = T_VerticalCompatibleDoor,
  .t_move_floor = T_MoveCompatibleFloor,
  .t_move_ceiling = T_MoveCompatibleCeiling,
  .t_build_pillar = NULL, // not used
  .t_plat_raise = T_CompatiblePlatRaise,
  .ev_teleport = EV_HereticTeleport,
  .player_thrust = P_HereticPlayerThrust,
  .build_mobj_thing_id_list = NULL, // not used
  .add_mobj_thing_id = NULL, // not used
  .remove_mobj_thing_id = NULL, // not used
  .iterate_spechit = P_IterateCompatibleSpecHit,
  .mapthing_size = sizeof(doom_mapthing_t),
  .maplinedef_size = sizeof(doom_maplinedef_t),
  .mt_push = -1,
  .mt_pull = -1,
  .dn_polyanchor = -1,
  .dn_polyspawn_start = -1,
  .dn_polyspawn_hurt = -1,
  .dn_polyspawn_end = -1,
  .visibility = VF_HERETIC,
};

static const map_format_t doom_map_format = {
  .zdoom = false,
  .hexen = false,
  .polyobjs = false,
  .acs = false,
  .thing_id = false,
  .mapinfo = false,
  .sndseq = false,
  .sndinfo = false,
  .animdefs = false,
  .doublesky = false,
  .map99 = false,
  .lax_monster_activation = true,
  .generalized_mask = ~31,
  .switch_activation = 0, // not used
  .init_sector_special = P_SpawnCompatibleSectorSpecial,
  .player_in_special_sector = P_PlayerInCompatibleSector,
  .mobj_in_special_sector = P_MobjInCompatibleSector,
  .spawn_scroller = P_SpawnCompatibleScroller,
  .spawn_friction = P_SpawnCompatibleFriction,
  .spawn_pusher = P_SpawnCompatiblePusher,
  .spawn_extra = P_SpawnCompatibleExtra,
  .cross_special_line = P_CrossCompatibleSpecialLine,
  .shoot_special_line = P_ShootCompatibleSpecialLine,
  .test_activate_line = NULL, // not used
  .execute_line_special = NULL, // not used
  .post_process_line_special = P_PostProcessCompatibleLineSpecial,
  .post_process_sidedef_special = P_PostProcessCompatibleSidedefSpecial,
  .animate_surfaces = P_AnimateCompatibleSurfaces,
  .check_impact = P_CheckCompatibleImpact,
  .translate_line_flags = P_TranslateCompatibleLineFlags,
  .apply_sector_movement_special = P_ApplyCompatibleSectorMovementSpecial,
  .t_vertical_door = T_VerticalCompatibleDoor,
  .t_move_floor = T_MoveCompatibleFloor,
  .t_move_ceiling = T_MoveCompatibleCeiling,
  .t_build_pillar = NULL, // not used
  .t_plat_raise = T_CompatiblePlatRaise,
  .ev_teleport = EV_CompatibleTeleport,
  .player_thrust = P_CompatiblePlayerThrust,
  .build_mobj_thing_id_list = NULL, // not used
  .add_mobj_thing_id = NULL, // not used
  .remove_mobj_thing_id = NULL, // not used
  .iterate_spechit = P_IterateCompatibleSpecHit,
  .mapthing_size = sizeof(doom_mapthing_t),
  .maplinedef_size = sizeof(doom_maplinedef_t),
  .mt_push = MT_PUSH,
  .mt_pull = MT_PULL,
  .dn_polyanchor = -1,
  .dn_polyspawn_start = -1,
  .dn_polyspawn_hurt = -1,
  .dn_polyspawn_end = -1,
  .visibility = VF_DOOM,
};

void dsda_ApplyZDoomMapFormat(void) {
  map_format = zdoom_in_hexen_map_format;

  if (!mbf21)
    I_Error("You must use complevel 21 when playing doom-in-hexen format maps.");

  dsda_MigrateMobjInfo();
}

void dsda_ApplyDefaultMapFormat(void) {
  if (hexen)
    map_format = hexen_map_format;
  else if (heretic)
    map_format = heretic_map_format;
  else
    map_format = doom_map_format;

  if (dsda_Flag(dsda_arg_mapinfo) && !map_format.mapinfo)
    map_format.mapinfo = W_LumpNameExists("MAPINFO");

  dsda_MigrateMobjInfo();
}
