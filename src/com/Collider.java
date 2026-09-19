package com;

// The static geometry a body is tested against: quad and triangle soup in a
// body's own room, with the vertex data kept as shorts and scaled by scale8.
// Finding contacts in it is the solver's job; this only describes it.

public final class Collider {
	public short[] verts;
	public short[] pols;
	public short[] norms;
	public int quads;
	public int tris;
	public int scale8;
	public int offX, offY, offZ;
}
