import java.util.*;

/**
* Union-Find Data structure.
* By calling the makeUnionFind(S) a one element set is made for every object s contained in S
* Objects are stored within Records which are put in trees. A Set can be recognized 
* by a top level record (with a parent pointing to null)
*
* This not a general "Set handling" class, but made for being used with for example
* Kruskal's Algorithm. 
* See http://en.wikipedia.org/wiki/Disjoint-set_data_structure for more info!
* -------------------------------------
* Implements makeUnionFind(S) in O(n), Union in O(log n), or O(1) if you a record representing a set,
* and find(u) called n times yield an amortized running time of per calling of O(a(n)) ~= O(1).
* 
**/


public class UnionFind<E> {
	
	/* Record class. Defines a set, or a record within a set */
	public class Record<E> {
		private int size;
		private E name;
		private Record<E> parent = null;
		
		public Record(E name) {
			this.name = name;
			size = 1;
		}
		public void setParent(Record<E> parent) {
			this.parent = parent;
		}
		public E getName() {
			return name;
		}
		private int getSize() {
			return size;
		}
		private void addToSize(int add) {
			size += add;
		}
		public Record<E> getParent() {
			return parent;
		}
		@SuppressWarnings("unchecked")
		public boolean equals(Object obj) {
			if(obj == null || getClass() != obj.getClass() ) {
				return false;
			}
			else {
				Record<E> o = (Record<E>) obj;
				return name.equals(o.getName());
			}
		}
	}
	/* data member - ArrayList containing all the records */	
	private ArrayList<Record<E>> records = new ArrayList<Record<E>>();
	
	/* Initializes all sets, one for every element in list set */
	public void makeUnionFind(List<E> Set) {
		for(E it : Set)
			records.add(new Record<E>(it));
	}
	
	public ArrayList<Record<E>> getRecords() {
		return records;
	}
	
	/* "Unionizes two sets */
	public void Union(Record<E> x, Record<E> y) {
			Record<E> xroot = find(x);
			Record<E> yroot = find(y);
		
			if(xroot.getSize() > yroot.getSize()) {
				yroot.setParent(xroot);
				xroot.addToSize(yroot.getSize());
			}
			else {
				xroot.setParent(yroot);
				yroot.addToSize(xroot.getSize());
			}
	}
	
	/* Given a records returns the top-record that represents the set 
	 * containing that record. Re-links the given record to the top-record (Path compression,
	 * the key to gain the amortized running time).
	 **/
	public Record<E> find(Record<E> rec) {
		if(rec.getParent() == null)
			return rec;
		else {
			rec.setParent(find(rec.getParent()));
			return rec.getParent();
		
		}
	}
	
	/* Checks if to records are in the same set. */
	public boolean sameSet(Record<E> r1, Record<E> r2) {
		return find(r1).equals(find(r2));
	}

}