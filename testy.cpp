class bool_wrapper
{
	bool x;
	public:
		bool_wrapper(bool x): x(x)
		{
		}
		operator const bool&() const {return x;}
};

int main()
{
	bool_wrapper b(true);
	bool b1 = b;
}