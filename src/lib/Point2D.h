#ifndef POINT_2D_H
#define POINT_2D_H

class Point2D {
	private:
		int x;
		int y;

	protected:

	public:	
		Point2D(int x = 0, int y = 0);		
		~Point2D();

		int getX() const;		
		int getY() const;

		void setX(int x);		
		void setY(int y);		
		void set(int x, int y);

		bool operator<(const Point2D& other) const;
};

#endif
