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
		public int getSize() {
			return size;
		}
		public void addToSize(int add) {
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