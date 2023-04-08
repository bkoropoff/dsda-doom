//
// Copyright(C) 2023 Brian Koropoff
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
//	BSP Analysis
//

#include <assert.h>
#include <math.h>
#include <stdlib.h>

#include "bsp.h"
#include "dgeom.h"
#include "lprintf.h"
#include "r_state.h"
#include "z_zone.h"
#include "m_bbox.h"
#include "gl_struct.h"
#include "p_setup.h"
#include "w_wad.h"
#include "r_bsp.h"
#include "r_sky.h"
#include "dsda/utility.h"
#include "dsda/data_organizer.h"
#include "dsda/workpool.h"

#include "ajbsp/glue.h"

// Calibrated to cover deep "fake" sectors in maps encountered in practice while
// being as efficient as possible
#define BLEED_THROUGH_DEPTH_LIMIT 3

// Iterator over perimeter of chunk
typedef struct piter_s
{
  // Chunk being iterated
  chunkmeta_t* chunkmeta;
  // Front subsector
  subsector_t* frontsub;
  // Front segment
  seg_t* frontseg;
  // Front segment
  segmeta_t* frontsegmeta;
  // Front side (NULL if both front and back segs are minisegs)
  side_t* frontside;
  // Back equivalents to the above
  subsector_t* backsub;
  seg_t* backseg;
  segmeta_t* backsegmeta;
  side_t* backside;
  // Back chunk, sector for convenience
  chunkmeta_t* backchunkmeta;
  sector_t* backsector;
  // Internal
  int i;
} piter_t;

// Bleed queue
typedef struct
{
  void** chunks;
  unsigned int count;
  unsigned int cap;
} bleedqueue_t;

// Temporary bleed before being added to global array
typedef struct
{
  bleed_t bleed;
  gl_chunk_t* source;
} tempbleed_t;

// Temporary bleed array
typedef struct
{
  tempbleed_t* bleeds;
  unsigned int count;
  unsigned int cap;
  unsigned int start;
} bleeds_t;

// Bleed worker local state
typedef struct
{
  // Queue of chunks to add bleed for
  bleedqueue_t queue;
  // Temporary bleeds
  bleeds_t bleeds;
  // Marked chunks
  byte mark[];
} bleedlocal_t;

dsda_gl_rstate_t dsda_gl_rstate = {0};

static unsigned int capchunks = 0;
static unsigned int capperims = 0;
static unsigned int cappaths = 0;
static unsigned int capbleeds = 0;
static unsigned int numrsegs = 0;

static const unsigned long long nan_pattern[2] = {0x7ff8000000000000ULL,
                                                  0x7ff8000000000000ULL};

static dboolean annotated = false;

// Chunk perimeter iteration.  Use like:
//
// for (IterPerimeter(&iter, sub);
//      IterPerimeterValid(&iter);
//      IterPerimeterNext(&iter)) ...

static side_t* OppositeSide(line_t* l, side_t* s)
{
  unsigned short num;

  if (l->sidenum[0] == s - sides)
    num = l->sidenum[1];
  else
    num = l->sidenum[0];

  return num == NO_INDEX ? NULL : &sides[num];
}

static inline dboolean IterPerimeterValid(const piter_t* iter)
{
  return iter->i != iter->chunkmeta->numperim;
}

static void IterPerimeterNext(piter_t* iter)
{
  iter->i++;
  if (IterPerimeterValid(iter))
  {
    chunkmeta_t* chunkmeta = iter->chunkmeta;

    iter->frontseg = dsda_gl_rstate.perims[chunkmeta->firstperim + iter->i];
    iter->frontsegmeta = &dsda_gl_rstate.segmeta[iter->frontseg - gl_rstate.segs];
    iter->frontsub = iter->frontsegmeta->subsector;

    assert(iter->frontsub->chunk == chunkmeta - dsda_gl_rstate.chunkmeta);

    if (!iter->frontsegmeta->partner)
    {
      iter->backseg = NULL;
      iter->backsegmeta = NULL;
      iter->backsub = NULL;
      iter->backchunkmeta = NULL;
      iter->backsector = NULL;
    }
    else
    {
      iter->backseg = iter->frontsegmeta->partner;
      iter->backsegmeta =
          &dsda_gl_rstate.segmeta[iter->frontsegmeta->partner - gl_rstate.segs];
      iter->backsub = iter->backsegmeta->subsector;
      iter->backchunkmeta = &dsda_gl_rstate.chunkmeta[iter->backsub->chunk];
      iter->backsector = iter->backsub->sector;
    }

    // Resolve sides
    if (iter->frontseg && iter->frontseg->linedef)
    {
      iter->frontside = iter->frontseg->sidedef;
      iter->backside = OppositeSide(iter->frontseg->linedef, iter->frontside);
    }
    else if (iter->backseg && iter->backseg->linedef)
    {
      // FIXME: can ajbsp generate miniseg/normal seg partners?
      iter->frontside = OppositeSide(iter->frontseg->linedef, iter->frontside);
      iter->backside = iter->frontseg->sidedef;
    }
    else
    {
      iter->frontside = NULL;
      iter->backside = NULL;
    }
  }
}

static inline void IterPerimeter(piter_t* iter, chunkmeta_t* chunkmeta)
{
  iter->chunkmeta = chunkmeta;
  iter->i = -1;
  IterPerimeterNext(iter);
}

// Pass 1 - load/build GL nodes if map did not provide them

static void LoadGLVertices(void* data, unsigned int size)
{
  typedef struct
  {
    int32_t x, y;
  } PACKEDATTR glvert5_t;

  glvert5_t* raw = (glvert5_t*) ((char*) data + 4);
  unsigned int count = (size > 4 ? size - 4 : 0) / sizeof(*raw);
  unsigned int i;

  gl_rstate.numvertexes = count;
  gl_rstate.vertexes = Z_Calloc(sizeof(*gl_rstate.vertexes), count);
  if (!gl_rstate.vertexes)
    I_Error("Not enough memory for GL vertices");

  for (i = 0; i < count; ++i)
  {
    glvert5_t* vraw = &raw[i];
    vertex_t* v = &gl_rstate.vertexes[i];

    v->x = v->px = LittleLong(vraw->x);
    v->y = v->py = LittleLong(vraw->y);
  }
}

static vertex_t* DecodeVert(uint32_t v)
{
    v = LittleLong(v);
    if (v & 0x80000000)
    {
      v &= ~0x80000000;
      if (v >= gl_rstate.numvertexes)
        I_Error("LoadGLSegs: invalid new vertex %u", v);
      return &gl_rstate.vertexes[v];
    }
    if (v >= numvertexes)
      I_Error("LoadGLSegs: invalid original vertex %u", v);
    return &vertexes[v];
}

static void LoadGLSegs(void* data, unsigned int size)
{
  typedef struct
  {
    uint32_t start;
    uint32_t end;
    uint16_t linedef;
    uint16_t side;
    uint32_t partner;
  } PACKEDATTR glseg5_t;

  glseg5_t* raw = (glseg5_t*) data;
  unsigned int count = size / sizeof(*raw);
  unsigned int i;

  gl_rstate.numsegs = count;
  gl_rstate.segs = Z_Calloc(sizeof(*gl_rstate.segs), count);
  if (!gl_rstate.segs)
    I_Error("Not enough memory for GL segs");
  dsda_gl_rstate.segmeta = Z_Calloc(sizeof(*dsda_gl_rstate.segmeta), count);
  if (!dsda_gl_rstate.segmeta)
    I_Error("Not enough memory for seg metadata");
  gl_rstate.rsegs = Z_Calloc(sizeof(*gl_rstate.rsegs), count);
  if (!gl_rstate.rsegs)
    I_Error("Not enough memory for GL rsegs");

  for (i = 0; i < count; ++i)
  {
    glseg5_t* sraw = &raw[i];
    seg_t* s = &gl_rstate.segs[i];
    segmeta_t* m = &dsda_gl_rstate.segmeta[i];
    uint16_t linedef;
    uint32_t partner;

    s->v1 = DecodeVert(sraw->start);
    s->v2 = DecodeVert(sraw->end);

    linedef = LittleShort(sraw->linedef);
    if (linedef == 0xFFFF)
      s->linedef = NULL;
    else if (linedef >= numlines)
      I_Error("LoadGLSegs: seg %u had invalid linedef %u", i, linedef);
    else
      s->linedef = &lines[linedef];

    partner = LittleLong(sraw->partner);
    if (partner == 0xFFFFFFFF)
      m->partner = NULL;
    else if (partner >= gl_rstate.numsegs)
      I_Error("LoadGLSegs: seg %u has invalid partner %u", i, partner);
    else
      m->partner = &gl_rstate.segs[partner];

    if (s->linedef)
    {
      if (sraw->side)
      {
        s->frontsector = s->linedef->backsector;
        s->backsector = s->linedef->frontsector;
        s->sidedef = &sides[s->linedef->sidenum[1]];
      }
      else
      {
        s->frontsector = s->linedef->frontsector;
        s->backsector = s->linedef->backsector;
        s->sidedef = &sides[s->linedef->sidenum[0]];
      }
    }

    m->flags = SEGF_NONE;
    // FIXME: probably not needed for gl... right?
    s->offset = s->pangle = 0;
  }
}

static void LoadGLSubsectors(void* data, unsigned int size)
{
  typedef struct
  {
    uint32_t num;
    uint32_t first;
  } PACKEDATTR glssect5_t;

  glssect5_t* raw = (glssect5_t*) data;
  unsigned int count = size / sizeof(*raw);
  unsigned int i;

  gl_rstate.numsubsectors = count;
  gl_rstate.subsectors = Z_Calloc(sizeof(*gl_rstate.subsectors), count);
  if (!gl_rstate.subsectors)
    I_Error("Not enough memory for GL subsectors");
  dsda_gl_rstate.submeta = Z_Calloc(sizeof(*dsda_gl_rstate.submeta), count);
  if (!dsda_gl_rstate.submeta)
    I_Error("Not enough memory for GL subsectors");

  for (i = 0; i < count; ++i)
  {
    glssect5_t* sraw = &raw[i];
    subsector_t* s = &gl_rstate.subsectors[i];
    submeta_t* m = &dsda_gl_rstate.submeta[i];

    s->numlines = LittleLong(sraw->num);
    s->firstline = LittleLong(sraw->first);

    if (s->firstline >= gl_rstate.numsegs ||
        s->numlines > gl_rstate.numsegs || // Ensure following does not overflow
        s->firstline + s->numlines > gl_rstate.numsegs)
      I_Error("LoadGLSubsectors: subsector %u has out-of-bounds segment range", i);

    s->poly = NULL;
    s->chunk = NO_CHUNK;
    s->sector = gl_rstate.segs[s->firstline].frontsector;
    if (!s->sector)
      I_Error("GL subsector had miniseg as first seg");

    m->flags = SUBF_NONE;
    m->q_next = NULL;
    m->chunk_next = NO_CHUNK;
  }

  gl_rstate.map_subsectors = Z_Calloc(sizeof(*gl_rstate.map_subsectors), count);
  if (!gl_rstate.map_subsectors)
    I_Error("Not enough memory for GL map subsectors");

}

static void LoadGLNodes(void* data, unsigned int size)
{
  typedef struct
  {
    int16_t x, y;
    int16_t dx, dy;
    int16_t boxes[2][4];
    uint32_t children[2];
  } PACKEDATTR glnode5_t;

  glnode5_t* raw = (glnode5_t*) data;
  unsigned int count = size / sizeof(*raw);
  unsigned int i;

  gl_rstate.numnodes = count;
  gl_rstate.nodes = Z_Calloc(sizeof(*gl_rstate.nodes), count);
  if (!gl_rstate.nodes)
    I_Error("Not enough memory for GL nodes");

  for (i = 0; i < count; ++i)
  {
    glnode5_t* nraw = &raw[i];
    node_t* n = &gl_rstate.nodes[i];
    int j, k;

    n->x = LittleShort(nraw->x) << FRACBITS;
    n->y = LittleShort(nraw->y) << FRACBITS;
    n->dx = LittleShort(nraw->dx) << FRACBITS;
    n->dy = LittleShort(nraw->dy) << FRACBITS;

    for (j = 0; j < 2; ++j)
      for (k = 0; k < 4; ++k)
        n->bbox[j][k] = LittleShort(nraw->boxes[j][k]) << FRACBITS;

    for (j = 0; j < 2; ++j)
    {
      uint32_t child = LittleLong(nraw->children[j]);

      if (child & NF_SUBSECTOR &&
          (child & ~NF_SUBSECTOR) >= gl_rstate.numsubsectors)
        I_Error("LoadGLNodes: node %u references invalid subsector %u", i,
                (child & ~NF_SUBSECTOR));
      else if (!(child & NF_SUBSECTOR) && child >= gl_rstate.numnodes)
        I_Error("LoadGLNodes: node %u references invalid child %u", i, child);
      n->children[j] = child;
    }
  }
}

static void LoadSideGLNodes()
{
  dsda_ajbsp_glnodes_t nodes;
  int lumpnum;
  const lumpinfo_t* info;
  const char* inpath;
  const char* basename;
  dsda_string_t outpath;

  lumpnum = W_GetNumForName(current_map_lump);
  info = W_GetLumpInfoByNum(lumpnum);
  inpath = info->wadfile->name;
  basename = dsda_BaseName(inpath);

  dsda_StringPrintF(&outpath, "%s/gl.%s", dsda_DataDir(), basename);

  if (!dsda_ajbsp_LoadGLNodes(inpath, outpath.string, current_map_lump, &nodes))
    I_Error("Could not build GL nodes");

  dsda_FreeString(&outpath);

  LoadGLVertices(nodes.vertexes, nodes.vsize);
  LoadGLSegs(nodes.segs, nodes.ssize);
  LoadGLSubsectors(nodes.subsectors, nodes.sssize);
  LoadGLNodes(nodes.nodes, nodes.nsize);

  Z_Free(nodes.vertexes);
  Z_Free(nodes.segs);
  Z_Free(nodes.subsectors);
  Z_Free(nodes.nodes);
}

static void InitGLNodes()
{
#if 0
  // Not ready for this yet
  if (use_gl_rstate.nodes)
  {
    // We already have GL nodes
    gl_rstate.vertexes = vertexes;
    gl_rstate.numvertexes = numvertexes;
    gl_rstate.segs = segs;
    gl_rstate.numsegs = numsegs;
    gl_rstate.subsectors = subsectors;
    gl_rstate.numsubsectors = numsubsectors;
    gl_rstate.map_subsectors = map_subsectors;
    gl_rstate.nodes = nodes;
    gl_rstate.numnodes = numnodes;
    return;
  }
#endif

  LoadSideGLNodes();
}

// Pass 2: mark map irregularities

static void AnnotateInit(void)
{
  int i, j;
  subsector_t* sub;
  segmeta_t* segmeta;

  for (i = 0; i < gl_rstate.numsubsectors; ++i)
  {
    sub = &gl_rstate.subsectors[i];

    for (j = 0; j < sub->numlines; ++j)
    {
      segmeta = &dsda_gl_rstate.segmeta[sub->firstline + j];
      segmeta->subsector = sub;
    }
  }
}

// Determine if subsector is degenerate (has fewer than 3 distinct segments)
// FIXME: does ajbsp even generate these?
static dboolean SubsectorIsDegenerate(subsector_t* sub)
{
  int i;
  seg_t* first = &gl_rstate.segs[sub->firstline];
  dboolean have_second = false;
  dline_t dfirst = dgeom_DLineFromSeg(first);
  dline_t dsecond;

  for (i = 1; i < sub->numlines; ++i)
  {
    seg_t* seg = &gl_rstate.segs[sub->firstline + i];
    dline_t dline = dgeom_DLineFromSeg(seg);

    if (!dgeom_LinesCoincide(&dfirst, &dline, DGEOM_EPSILONR2))
    {
      if (!have_second)
      {
        dsecond = dline;
        have_second = true;
      }
      else if (!dgeom_LinesCoincide(&dsecond, &dline, DGEOM_EPSILONR2))
        return false;
    }
  }

  return true;
}

static void AnnotateDegenerateSubsectors(void)
{
  int i;

  // Now we can mark degenerate subsectors
  for (i = 0; i < gl_rstate.numsubsectors; ++i)
  {
    subsector_t* sub = &gl_rstate.subsectors[i];
    submeta_t* submeta = &dsda_gl_rstate.submeta[i];

    if (SubsectorIsDegenerate(sub))
      submeta->flags |= SUBF_DEGENERATE;
  }
}

// Detect segs involved in rendering hacks
static void AnnotateHackedSegs(void)
{
  int i;

  for (i = 0; i < gl_rstate.numsegs; ++i)
  {
    seg_t* seg = &gl_rstate.segs[i];
    segmeta_t* segmeta = &dsda_gl_rstate.segmeta[i];
    seg_t* partner = segmeta->partner;
    segmeta_t* psegmeta;
    dboolean mismatch_case, miniseg_case;

    if (!partner)
      continue;

    psegmeta = &dsda_gl_rstate.segmeta[partner - gl_rstate.segs];

    // Case 1: seg's linedef has the same sector on both sides, but partner
    // seg's sector doesn't match
    mismatch_case = seg->linedef &&
                    seg->linedef->frontsector == seg->linedef->backsector &&
                    psegmeta->subsector->sector != seg->linedef->backsector;

    // Case 2: minisegs separating different sectors.
    // Minisegs ordinarly divide subsectors in the same sector, but self-referencing
    // sector tricks can sometimes give rise to this case.
    miniseg_case = (!seg->linedef || !partner->linedef) &&
                   segmeta->subsector->sector != psegmeta->subsector->sector;

    if (mismatch_case || miniseg_case)
    {
      submeta_t* submeta =
          &dsda_gl_rstate.submeta[segmeta->subsector - gl_rstate.subsectors];
      submeta_t* psubmeta =
          &dsda_gl_rstate.submeta[psegmeta->subsector - gl_rstate.subsectors];

      segmeta->flags |= SEGF_HACKED;
      psegmeta->flags |= SEGF_HACKED;
      submeta->flags |= SUBF_HACKED;
      psubmeta->flags |= SUBF_HACKED;
    }
  }
}


// Pass 3: compute chunks

static void PushChunk(sector_t* sector, gl_chunk_t** compout, chunkmeta_t** metaout)
{
  gl_chunk_t* chunk;
  chunkmeta_t* chunkmeta;

  if (gl_rstate.numchunks == capchunks)
  {
    if (capchunks == 0)
      capchunks = 256;
    else
      capchunks *= 2;
    gl_rstate.chunks = Z_Realloc(
        gl_rstate.chunks, sizeof(*gl_rstate.chunks) * capchunks);
    if (!gl_rstate.chunks)
      I_Error("Could not allocate memory for chunks");
    dsda_gl_rstate.chunkmeta = Z_Realloc(
        dsda_gl_rstate.chunkmeta, sizeof(*dsda_gl_rstate.chunkmeta) * capchunks);
    if (!dsda_gl_rstate.chunkmeta)
      I_Error("Could not allocate memory for chunk metadata");
  }

  chunk = &gl_rstate.chunks[gl_rstate.numchunks];
  memset(chunk, 0, sizeof(*chunk));
  chunk->sector = sector;
  chunk->firstbleed = -1;
  chunk->floorthrough.depth = INT_MAX;
  chunk->ceilingthrough.depth = INT_MAX;
  chunk->floorunder.depth = INT_MAX;
  chunk->ceilingunder.depth = INT_MAX;
  chunk->floorover.depth = INT_MAX;
  chunk->ceilingover.depth = INT_MAX;

  chunkmeta = &dsda_gl_rstate.chunkmeta[gl_rstate.numchunks++];
  memset(chunkmeta, 0, sizeof(*chunkmeta));
  chunkmeta->firstpoint = -1;
  chunkmeta->subsectors = -1;

  *compout = chunk;
  *metaout = chunkmeta;
}

static void AnnotateChunks(void)
{
  int i, j;

  for (i = 0; i < gl_rstate.numsubsectors; ++i)
  {
    subsector_t* sub = &gl_rstate.subsectors[i];
    submeta_t* submeta = &dsda_gl_rstate.submeta[i];
    submeta_t* queue = NULL;
    gl_chunk_t* comp;
    chunkmeta_t* chunkmeta;

    if (sub->chunk != NO_CHUNK)
      continue;

    // Begin a new chunk starting with this subsector
    PushChunk(sub->sector, &comp, &chunkmeta);
    chunkmeta->subsectors = sub - gl_rstate.subsectors;
    sub->chunk = comp - gl_rstate.chunks;

    // Hacked subsectors are subject to bleed-through
    if (submeta->flags & SUBF_HACKED)
      chunkmeta->flags |= CHUNKF_BLEED_THROUGH;

    // Degenerate subsectors get their own chunk so they
    // don't muck up perimeters
    // FIXME: is this necessary with ajbsp?
    if (submeta->flags & SUBF_DEGENERATE)
    {
      chunkmeta->flags |= CHUNKF_DEGENERATE;
      continue;
    }

    // Recursively mark reachable subsectors with the chunk
    queue = submeta;

    while (queue)
    {
      submeta = queue;
      sub = &gl_rstate.subsectors[submeta - dsda_gl_rstate.submeta];
      queue = queue->q_next;
      submeta->q_next = NULL;

      for (j = 0; j < sub->numlines; ++j)
      {
        segmeta_t* segmeta = &dsda_gl_rstate.segmeta[sub->firstline + j];
        seg_t* partner = segmeta->partner;
        segmeta_t* psegmeta;
        subsector_t* adjss;
        submeta_t* adjsm;

        if (!partner)
          continue;

        psegmeta = &dsda_gl_rstate.segmeta[partner - gl_rstate.segs];

        adjss = psegmeta->subsector;
        adjsm = &dsda_gl_rstate.submeta[adjss - gl_rstate.subsectors];

        if (adjss->chunk != NO_CHUNK || adjss->sector != comp->sector ||
            adjsm->flags & SUBF_DEGENERATE)
          continue;

        adjss->chunk = comp - gl_rstate.chunks;
        adjsm->chunk_next = chunkmeta->subsectors;
        chunkmeta->subsectors = adjsm - dsda_gl_rstate.submeta;

        if (adjsm->flags & SUBF_HACKED)
          chunkmeta->flags |= CHUNKF_BLEED_THROUGH;

        adjsm->q_next = queue;
        queue = adjsm;
      }
    }
  }

  // This doesn't seem to be necessary with ajbsp so far because its
  // assignment of subsectors to sectors (via the first seg) is better
#if 0
  // Break up chunks subject to bleed-through because each subsector
  // might need to render with different flats due to bleed effects
  for (i = 0; i < numchunks; ++i)
  {
    chunk_t* comp = &chunks[i];
    subsector_t* sub;
    subsector_t* next;

    if (!(comp->flags & CHUNKF_BLEED_THROUGH) || !comp->subsectors->comp_next)
      continue;

    sub = comp->subsectors->comp_next;
    comp->subsectors->comp_next = NULL;
    for (; sub; sub = next)
    {
      next = sub->comp_next;
      sub->comp_next = NULL;
      comp = PushChunk(sub->sector);
      sub->chunk = comp - chunks;
      comp->subsectors = sub;
      comp->flags |= CHUNKF_BLEED_THROUGH;
    }
  }
#endif

  // We know the number of chunks, this can now be allocated
  gl_rstate.deferred =
      Z_Calloc(sizeof(*gl_rstate.deferred), gl_rstate.numchunks);
  if (!gl_rstate.deferred)
    I_Error("Could not allocate memory for deferred flats queue");
}

// Pass 4: derive chunk perimeters from subsectors

static void PushPerim(seg_t* seg)
{
  if (dsda_gl_rstate.numperims == capperims)
  {
    if (capperims == 0)
      capperims = 256;
    else
      capperims *= 2;
    dsda_gl_rstate.perims = Z_Realloc(
        dsda_gl_rstate.perims, sizeof(*dsda_gl_rstate.perims) * capperims);
    if (!dsda_gl_rstate.perims)
      I_Error("Could not allocate memory for adjacency");
  }
  dsda_gl_rstate.perims[dsda_gl_rstate.numperims++] = seg;
}

static void AddChunkPerimeter(chunkmeta_t* chunkmeta, seg_t* seg)
{
  chunkmeta->numperim++;
  PushPerim(seg);
}

static void AnnotateChunkPerimeter(chunkmeta_t* chunkmeta)
{
  subsector_t* sub;
  submeta_t* submeta;
  int subnum;
  int i;
  dboolean interior = true;

  chunkmeta->firstperim = dsda_gl_rstate.numperims;

  for (subnum = chunkmeta->subsectors; subnum != -1;
       subnum = submeta->chunk_next)
  {
    sub = &gl_rstate.subsectors[subnum];
    submeta = &dsda_gl_rstate.submeta[subnum];

    for (i = 0; i < sub->numlines; ++i)
    {
      seg_t* seg = &gl_rstate.segs[sub->firstline + i];
      segmeta_t* segmeta = &dsda_gl_rstate.segmeta[sub->firstline + i];
      segmeta_t* psegmeta =
          segmeta->partner
              ? &dsda_gl_rstate.segmeta[segmeta->partner - gl_rstate.segs]
              : NULL;

      assert(segmeta->subsector->chunk ==
             chunkmeta - dsda_gl_rstate.chunkmeta);

      if (!psegmeta ||
          psegmeta->subsector->chunk != chunkmeta - dsda_gl_rstate.chunkmeta)
        AddChunkPerimeter(chunkmeta, seg);

      if (!psegmeta)
        interior = false;
    }
  }

  // Note purely interior chunks, which can speed up some bleed calculations
  if (interior)
    chunkmeta->flags |= CHUNKF_INTERIOR;
}

static void AnnotateChunkPerimeters(void)
{
  int i = 0;

  for (i = 0; i < gl_rstate.numchunks; ++i)
    AnnotateChunkPerimeter(&dsda_gl_rstate.chunkmeta[i]);
}

// Pass 5 -- try to generate polygon paths for chunks

static void PushPath(chunkmeta_t* chunkmeta, const void* point)
{
  if (dsda_gl_rstate.numpaths == cappaths)
  {
    if (cappaths == 0)
      cappaths = 256;
    else
      cappaths *= 2;
    dsda_gl_rstate.paths = Z_Realloc(dsda_gl_rstate.paths,
                                     sizeof(*dsda_gl_rstate.paths) * cappaths);
    if (!dsda_gl_rstate.paths)
      I_Error("Could not allocate memory for outlines");
  }

  if (chunkmeta->firstpoint == -1)
    chunkmeta->firstpoint = dsda_gl_rstate.numpaths;
  chunkmeta->numpoints++;

  // Avoid putting a NAN in a double local or rvalue at any point, as it's not
  // safe with -ffast-math
  memcpy(&dsda_gl_rstate.paths[dsda_gl_rstate.numpaths++], point,
         sizeof(*dsda_gl_rstate.paths));
}

static dboolean FindChunkPath(chunkmeta_t* chunkmeta)
{
  piter_t iter;
  segflags_t* goalflags;
  int goal = -1;
  dboolean restart = false;

  for (;;)
  {
    if (goal != -1 && dsda_gl_rstate.numpaths - 1 != goal &&
        dgeom_PointsEqual(&dsda_gl_rstate.paths[dsda_gl_rstate.numpaths - 1],
                          &dsda_gl_rstate.paths[goal], DGEOM_EPSILON))
    {
      // Mark end-of-contour
      PushPath(chunkmeta, nan_pattern);
      goal = -1;
    }

    for (IterPerimeter(&iter, chunkmeta);
        IterPerimeterValid(&iter);
        IterPerimeterNext(&iter))
    {
      dline_t l;
      segflags_t* flags;

      if (iter.frontsegmeta->flags & SEGF_MARK)
        continue;

      l = dgeom_DLineFromSeg(iter.frontseg);
      flags = &iter.frontsegmeta->flags;

      if (goal == -1 ||
          dgeom_PointsEqual(&dsda_gl_rstate.paths[dsda_gl_rstate.numpaths - 1],
                            &l.start, DGEOM_EPSILON))
      {
        if (goal == -1)
        {
          // Start a new search from here
          goal = dsda_gl_rstate.numpaths;
          goalflags = flags;
          PushPath(chunkmeta, &l.start);
        }

        PushPath(chunkmeta, &l.end);
        *flags |= SEGF_MARK;

        restart = true;
        break;
      }
    }

    if (restart)
    {
      restart = false;
      continue;
    }

    if (goal != -1)
    {
      // Failed to use seg, mark it for debugging purposes
      *goalflags |= SEGF_ORPHAN;
      // Remove all added points
      chunkmeta->numpoints -= dsda_gl_rstate.numpaths - goal;
      dsda_gl_rstate.numpaths = goal;
      // Clear goal and keep going
      goal = -1;
      continue;
    }

    // If we get to this point, we didn't find any more segs
    // to start a contour, so we're done
    break;
  }

  // FIXME: be more discerning about when we succeeded, e.g.
  // by counting number of orphan/unused segs and comparing it against
  // total, or figuring out which segs are safe to not use
  return chunkmeta->numpoints != 0;
}

// Try to find path for chunk
static void AnnotateChunkPath(chunkmeta_t* chunkmeta)
{
  piter_t iter;

  if (!FindChunkPath(chunkmeta))
  {
    // Throw away any contours we found so far on failure
    dsda_gl_rstate.numpaths -= chunkmeta->numpoints;
    chunkmeta->numpoints = 0;
  }

  // Clear marks
  for (IterPerimeter(&iter, chunkmeta);
      IterPerimeterValid(&iter);
      IterPerimeterNext(&iter))
    iter.frontsegmeta->flags &= ~SEGF_MARK;
}

static void AnnotateChunkPaths(void)
{
  int i;

  for (i = 0; i < gl_rstate.numchunks; ++i)
  {
    chunkmeta_t* chunkmeta = &dsda_gl_rstate.chunkmeta[i];

    // Degenerate chunks will obviously not yield a useful path.
    if (chunkmeta->flags & CHUNKF_DEGENERATE)
      continue;

    AnnotateChunkPath(chunkmeta);
  }
}

// Pass 6 - compute bleed effects

static void PushGlobalBleed(bleed_t* bleed)
{
  if (gl_rstate.numbleeds == capbleeds)
  {
    if (capbleeds == 0)
      capbleeds = 256;
    else
      capbleeds *= 2;
    gl_rstate.bleeds =
        Z_Realloc(gl_rstate.bleeds, sizeof(*gl_rstate.bleeds) * capbleeds);
    if (!gl_rstate.bleeds)
      I_Error("Could not allocate memory for bleeds");
  }

  gl_rstate.bleeds[gl_rstate.numbleeds++] = *bleed;
}

static void PushBleed(bleeds_t* bleeds, gl_chunk_t* source,
                      gl_chunk_t* target, bleedtype_t type, int depth)
{
  tempbleed_t* bleed;

  if (bleeds->count == bleeds->cap)
  {
    if (bleeds->cap == 0)
      bleeds->cap = 256;
    else
      bleeds->cap *= 2;
    // Zone memory functions are not thread-safe
    bleeds->bleeds = realloc(bleeds->bleeds, sizeof(*bleeds->bleeds) * bleeds->cap);
    if (!bleeds->bleeds)
      I_Error("Could not allocate memory for bleeds");
  }

  bleed = &bleeds->bleeds[bleeds->count++];
  bleed->bleed.target = target;
  bleed->bleed.type = type;
  bleed->bleed.depth = depth;
  bleed->source = source;
}

static void PushQueue(bleedqueue_t* queue, chunkmeta_t* chunkmeta)
{
  if (queue->count == queue->cap)
  {
    if (queue->cap == 0)
      queue->cap = 256;
    else
      queue->cap *= 2;
    // Zone memory functions are not thread-safe
    queue->chunks = realloc(
        queue->chunks, sizeof(*queue->chunks) * queue->cap);
    if (!queue->chunks)
      I_Error("Could not allocate memory for adjacency");
  }
  queue->chunks[queue->count++] = chunkmeta;
}

// Based on logic in gzdoom.
// See https://github.com/ZDoom/gzdoom/blob/6cf91d3/src/rendering/hwrenderer/scene/hw_renderhacks.cpp#L539
static dboolean BleedOverTransitive(chunkmeta_t* chunkmeta, dboolean ceiling,
                                    fixed_t height, bleedlocal_t* bl)
{
  piter_t iter;
  dboolean result = true;

  if (!(chunkmeta->flags & CHUNKF_INTERIOR))
    return false;

  bl->mark[chunkmeta - dsda_gl_rstate.chunkmeta] = true;
  PushQueue(&bl->queue, chunkmeta);

  for (IterPerimeter(&iter, chunkmeta); IterPerimeterValid(&iter);
       IterPerimeterNext(&iter))
  {
    fixed_t backheight;

    // Skip single-sided lines
    if (!iter.backchunkmeta)
      continue;

    // Skip already-visited chunks
    if (bl->mark[iter.backchunkmeta - dsda_gl_rstate.chunkmeta])
      continue;

    // Skip degenerate chunks
    // FIXME: are degenerate chunks a problem with ajbsp?
    if (iter.backchunkmeta->flags & CHUNKF_DEGENERATE)
      continue;

    backheight =
        ceiling ? iter.backsector->ceilingheight : iter.backsector->floorheight;

    // Bleeding stops at textured segs
    if (iter.backside &&
        ((ceiling && iter.backside->toptexture != NO_TEXTURE) ||
         (!ceiling && iter.backside->bottomtexture != NO_TEXTURE)))
      continue;

    // Bleeding stops at another chunk which would produce an
    // equivalent bleed (to prevent rejection by the next test)
    if ((backheight == height &&
         (!iter.frontside ||
          (ceiling && iter.frontside->toptexture == NO_TEXTURE) ||
          (!ceiling && iter.frontside->bottomtexture == NO_TEXTURE))))
      continue;

    // Fail on bleeds that find a chunk with a height not suitable for the
    // bleed type.  This heuristic aborts bleeds that did not appear to be a
    // conscious attempt by the map maker to use flat bleeding effects, to
    // avoid false positives.
    if ((!ceiling && backheight >= height) ||
        (ceiling && backheight <= height))
    {
      result = false;
      break;
    }

    if (!BleedOverTransitive(iter.backchunkmeta, ceiling, height, bl))
    {
      result = false;
      break;
    }
  }

  return result;
}

// FIXME: deduplicate with previous function, although maybe it's already
// confusing enough as-is
static dboolean BleedUnderTransitive(chunkmeta_t* chunkmeta, dboolean ceiling,
                                     fixed_t height, bleedlocal_t* bl,
                                     dboolean fail_on_height_reject)
{
  piter_t iter;
  dboolean result = true;

  if (fail_on_height_reject && !(chunkmeta->flags & CHUNKF_INTERIOR))
    return false;

  bl->mark[chunkmeta - dsda_gl_rstate.chunkmeta] = true;
  PushQueue(&bl->queue, chunkmeta);

  for (IterPerimeter(&iter, chunkmeta); IterPerimeterValid(&iter);
       IterPerimeterNext(&iter))
  {
    fixed_t backheight;

    // Skip single-sided lines
    if (!iter.backsector)
      continue;

    // Skip already-visited chunks
    if (bl->mark[iter.backchunkmeta - dsda_gl_rstate.chunkmeta])
      continue;

    // Skip degenerate chunks
    // FIXME: are degenerate chunks a problem with ajbsp?
    if (iter.backchunkmeta->flags & CHUNKF_DEGENERATE)
      continue;

    backheight =
        ceiling ? iter.backsector->ceilingheight : iter.backsector->floorheight;

    // Bleeding stops at textured segs
    if (iter.frontside &&
        ((ceiling && iter.frontside->toptexture != NO_TEXTURE) ||
         (!ceiling && iter.frontside->bottomtexture != NO_TEXTURE)))
      continue;

    // Bleeding stops at another chunk which would produce an
    // equivalent bleed (to prevent rejection by the next test)
    if ((backheight == height &&
         (!iter.backside ||
          (ceiling && iter.backside->toptexture == NO_TEXTURE) ||
          (!ceiling && iter.backside->bottomtexture == NO_TEXTURE))))
      continue;

    // Fail on bleeds that find a chunk with a height not suitable for the
    // bleed type (unless !fail_on_height_reject).  This heuristic aborts bleeds
    // that did not appear to be a conscious attempt by the map maker to use
    // flat bleeding effects, to avoid false positives.
    if ((!ceiling && backheight <= height) ||
        (ceiling && backheight >= height))
    {
      if (!fail_on_height_reject)
        continue;
      result = false;
      break;
    }

    // Follow bleed recursively
    if (!BleedUnderTransitive(iter.backchunkmeta, ceiling, height, bl,
                              fail_on_height_reject))
    {
      result = false;
      break;
    }
  }

  return result;
}

static bleedtype_t BleedType(dboolean ceiling, dboolean over)
{
  if (ceiling && over)
    return BLEED_CEILING_OVER;
  else if (ceiling && !over)
    return BLEED_CEILING_UNDER;
  else if (!ceiling && over)
    return BLEED_FLOOR_OVER;
  else
    return BLEED_FLOOR_UNDER;
}

static void AddBleed(bleeds_t* bleeds, gl_chunk_t* source, gl_chunk_t* target,
                     bleedtype_t type, int depth)
{
  int i;

  // Avoid duplicates
  for (i = bleeds->start; i < bleeds->count; ++i)
  {
    tempbleed_t* bleed = &bleeds->bleeds[i];
    if (bleed->source == source && bleed->bleed.target == target &&
        bleed->bleed.type == type)
    {
      if (depth < bleed->bleed.depth)
        // Register shorter path for same bleed
        bleed->bleed.depth = depth;
      return;
    }
  }

  PushBleed(bleeds, source, target, type, depth);
}

static void BleedProcessQueue(chunkmeta_t* sourcemeta, bleedlocal_t* bl,
                              dboolean ceiling, dboolean over, int depth,
                              dboolean bleed)
{
  unsigned int i;

  for (i = 0; i < bl->queue.count; ++i)
  {
    gl_chunk_t* source;
    gl_chunk_t* target;
    chunkmeta_t* targetmeta = bl->queue.chunks[i];

    bl->mark[targetmeta - dsda_gl_rstate.chunkmeta] = false;

    if (!bleed)
      continue;

    source = GL_Chunk(sourcemeta - dsda_gl_rstate.chunkmeta);
    target = GL_Chunk(targetmeta - dsda_gl_rstate.chunkmeta);
    AddBleed(&bl->bleeds, source, target, BleedType(ceiling, over), depth);
  }

  bl->queue.count = 0;
}

static void BleedChunkPlane(chunkmeta_t* sourcemeta, chunkmeta_t* chunkmeta,
                                dboolean ceiling, int depth,
                                bleedlocal_t* bl)
{
  gl_chunk_t* source = GL_Chunk(sourcemeta - dsda_gl_rstate.chunkmeta);
  int height = ceiling ? source->sector->ceilingheight : source->sector->floorheight;
  piter_t iter;

  if (chunkmeta->flags & CHUNKF_DEGENERATE)
    return;

  bl->mark[chunkmeta - dsda_gl_rstate.chunkmeta] = true;

  for (IterPerimeter(&iter, chunkmeta); IterPerimeterValid(&iter);
       IterPerimeterNext(&iter))
  {
    fixed_t backheight;
    dboolean texture_check;
    dboolean height_check;

    // Skip single-sided lines
    if (!iter.backchunkmeta)
      continue;

    // Skip already-visited chunks
    if (bl->mark[iter.backchunkmeta - dsda_gl_rstate.chunkmeta])
      continue;

    // Skip degenerate chunks
    if (iter.backchunkmeta->flags & CHUNKF_DEGENERATE)
      continue;

    backheight =
        ceiling ? iter.backsector->ceilingheight : iter.backsector->floorheight;

    // Check for bleed over
    //
    //      Upper floor bleeds this way ->
    // -----+~~~~~~~~~~~~
    //      |
    //      |
    //      | <- No texture
    //      |
    //      |
    //      +------------
    //
    // Turn picture upside down for ceiling case
    texture_check =
        (iter.backside &&
         ((ceiling && iter.backside->toptexture == NO_TEXTURE) ||
          (!ceiling && iter.backside->bottomtexture == NO_TEXTURE)));
    height_check =
        // Tagged sectors could change height, so assume they pass
        source->sector->tag || iter.backsector->tag ||
        (!ceiling && backheight < height) || (ceiling && backheight > height);
    if (texture_check && height_check)
    {
      dboolean bleed =
          BleedOverTransitive(iter.backchunkmeta, ceiling, height, bl);
      BleedProcessQueue(sourcemeta, bl, ceiling, true, depth, bleed);
    }

    // Check for bleed under
    //
    // ------------------+
    //                   |
    //                   |
    //                   | <- No texture
    //                   |
    //                   |
    // ~~~~~~~~~~~~~~~~~~+-----------
    //      <- Lower floor bleeds this way
    //
    // Turn picture upside down for ceiling case
    
    texture_check =
      // Always go through hacked segs so we include miniseg adjacencies
      // between sectors, which would otherwise not pass this check since
      // they have no sidedefs.  Maps with errant sidedef sectors
      // can produce scenarios that require this.
      iter.frontsegmeta->flags & SEGF_HACKED ||
        (iter.frontside &&
         ((ceiling && iter.frontside->toptexture == NO_TEXTURE) ||
          (!ceiling && iter.frontside->bottomtexture == NO_TEXTURE)));
    height_check =
        // Tagged sectors could change height, so assume they pass
        source->sector->tag || iter.backsector->tag ||
        (!ceiling && backheight > height) || (ceiling && backheight < height);
    if (texture_check && height_check)
    {
      dboolean bleed = BleedUnderTransitive(
          iter.backchunkmeta,
          ceiling,
          height,
          bl,
          // Be permissive crossing minisegs, fail on height mismatches
          // otherwise
          !!iter.frontside);
      BleedProcessQueue(sourcemeta, bl, ceiling, false, depth, bleed);
    }

    // Check for bleed-through to suspected "fake" sectors and recurse on them
    if (depth < BLEED_THROUGH_DEPTH_LIMIT &&
        iter.backchunkmeta->flags & CHUNKF_BLEED_THROUGH &&
        height == backheight)
    {
      bleedtype_t type = ceiling ? BLEED_CEILING_THROUGH : BLEED_FLOOR_THROUGH;
      gl_chunk_t* backcomp = GL_Chunk(iter.backchunkmeta - dsda_gl_rstate.chunkmeta);
      AddBleed(&bl->bleeds, source, backcomp, type, depth);
      BleedChunkPlane(sourcemeta, iter.backchunkmeta, ceiling, depth + 1, bl);
    }
  }
}

static void BleedChunk(chunkmeta_t* chunkmeta, bleedlocal_t* bl)
{
  bl->bleeds.start = bl->bleeds.count;

  BleedChunkPlane(chunkmeta, chunkmeta, false, 0, bl);
  memset(bl->mark, 0, sizeof(*bl->mark) * gl_rstate.numchunks);

  BleedChunkPlane(chunkmeta, chunkmeta, true, 0, bl);
  memset(bl->mark, 0, sizeof(*bl->mark) * gl_rstate.numchunks);
}

// Chunk chunks into this many per work item to amortize
// synchronization overhead
static const unsigned int chunk_size = 8;

static void BleedWorkCallback(void* item, void* local, void* context)
{
  chunkmeta_t* chunkmeta = (chunkmeta_t*) item;
  bleedlocal_t* bl = (bleedlocal_t*) local;
  unsigned offset = chunkmeta - dsda_gl_rstate.chunkmeta;
  unsigned int i;

  // Compute bleeds for all chunks in this chunk
  for (i = 0; i < chunk_size && i + offset < gl_rstate.numchunks; ++i)
    BleedChunk(&chunkmeta[i], bl);
}

static void BleedDestroyCallback(void* local, void* context)
{
  bleedlocal_t* bl = (bleedlocal_t*) local;
  unsigned int i;

  // Commit temporary bleeds to global array
  for (i = 0; i < bl->bleeds.count; ++i)
  {
    tempbleed_t* bleed = &bl->bleeds.bleeds[i];

    if (bleed->source->firstbleed == -1)
      bleed->source->firstbleed = gl_rstate.numbleeds;

    PushGlobalBleed(&bleed->bleed);
    bleed->source->numbleeds++;
  }

  // Zone memory functions are not thread-safe
  if (bl->bleeds.bleeds)
    free(bl->bleeds.bleeds);

  if (bl->queue.chunks)
    free(bl->queue.chunks);
}

static void AnnotateChunkBleeds(void)
{
  dsda_wp_t wp;
  dsda_wp_params_t wpp;
  int i;

  dsda_WorkPoolInitParams(&wpp);
  wpp.queuesize = gl_rstate.numchunks;
  wpp.work = BleedWorkCallback;
  wpp.destroy = BleedDestroyCallback;
  // Local state includes flexible array of dboolean for all chunks
  wpp.localsize =
      sizeof(bleedlocal_t) + sizeof(dboolean) * gl_rstate.numchunks;

  dsda_WorkPoolInit(&wp, &wpp);

  // Enqueue all chunks as work chunks
  for (i = 0; i < gl_rstate.numchunks; i += chunk_size)
    dsda_WorkPoolEnqueue(&wp, dsda_gl_rstate.chunkmeta + i);

  // Implicitly flushes
  dsda_WorkPoolDestroy(&wp);
}

// Pass 7 -- generate optimized renderable segment array
// Avoiding minisegs is important for cache efficiency in the render path

static void AnnotateSubsectorRSegs(subsector_t* sub)
{
  int i;

  sub->firstrseg = numrsegs;

  for (i = 0; i < sub->numlines; ++i)
  {
    seg_t* seg = &gl_rstate.segs[sub->firstline + i];

    if (!seg->linedef)
      continue;

    gl_rstate.rsegs[numrsegs++] = *seg;
    sub->numrsegs++;
  }
}

static void AnnotateRSegs(void)
{
  int i;

  for (i = 0; i < gl_rstate.numsubsectors; ++i)
    AnnotateSubsectorRSegs(&gl_rstate.subsectors[i]);
}

//
// Public interface
//

void dsda_AnnotateBSP(void)
{
  if (!annotated)
  {
    InitGLNodes();
    AnnotateInit();
    AnnotateDegenerateSubsectors();
    AnnotateHackedSegs();
    AnnotateChunks();
    AnnotateChunkPerimeters();
    AnnotateChunkPaths();
    AnnotateChunkBleeds();
    AnnotateRSegs();
    annotated = true;
  }
}

void dsda_ClearBSP(void)
{
  gl_rstate.numvertexes = 0;
  if (gl_rstate.vertexes && gl_rstate.vertexes != vertexes)
    Z_Free(gl_rstate.vertexes);

  gl_rstate.numsegs = 0;
  if (gl_rstate.segs && gl_rstate.segs != segs)
    Z_Free(gl_rstate.segs);
  if (dsda_gl_rstate.segmeta)
    Z_Free(dsda_gl_rstate.segmeta);

  gl_rstate.numsubsectors = 0;
  if (gl_rstate.subsectors && gl_rstate.subsectors != subsectors)
    Z_Free(gl_rstate.subsectors);
  if (gl_rstate.map_subsectors)
    Z_Free(gl_rstate.map_subsectors);
  if (dsda_gl_rstate.submeta)
    Z_Free(dsda_gl_rstate.submeta);

  gl_rstate.numnodes = 0;
  if (gl_rstate.nodes && gl_rstate.nodes != nodes)
    Z_Free(gl_rstate.nodes);

  numrsegs = 0;
  if (gl_rstate.rsegs)
    Z_Free(gl_rstate.rsegs);

  dsda_gl_rstate.numperims = 0;
  capperims = 0;
  if (dsda_gl_rstate.perims)
    Z_Free(dsda_gl_rstate.perims);
  dsda_gl_rstate.perims = NULL;

  gl_rstate.numchunks = 0;
  capchunks = 0;
  if (gl_rstate.chunks)
    Z_Free(gl_rstate.chunks);
  gl_rstate.chunks = NULL;
  if (gl_rstate.deferred)
    Z_Free(gl_rstate.deferred);
  gl_rstate.deferred = NULL;
  if (dsda_gl_rstate.chunkmeta)
    Z_Free(dsda_gl_rstate.chunkmeta);

  dsda_gl_rstate.numpaths = 0;
  cappaths = 0;
  if (dsda_gl_rstate.paths)
    Z_Free(dsda_gl_rstate.paths);
  dsda_gl_rstate.paths = NULL;

  gl_rstate.numbleeds = 0;
  capbleeds = 0;
  if (gl_rstate.bleeds)
    Z_Free(gl_rstate.bleeds);
  gl_rstate.bleeds = NULL;

  annotated = false;
}

dboolean dsda_PointIsEndOfContour(const dpoint_t* p)
{
  // Avoid putting nan into a double local or rvalue at any point, as it's not
  // safe with -ffast-math
  return !memcmp(p, nan_pattern, sizeof(nan_pattern));
}
