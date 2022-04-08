template <typename T>
void Sort(T* array, size_t low, size_t high, std::function<int(T&, T&)>& comparer) noexcept
{
	if (!array)
		return;
	auto swap = [=](T* a, T* b) -> void {
		T t = *a;
		*a = *b;
		*b = t;
	};
	auto partition = [=](T* array, size_t low, size_t high) -> double{
		T pivot = array[high];
		size_t i = low - 1;
		for (size_t j = low; j <= high - 1; j++)
		{
			if ((comparer(array[j], pivot) < 0) || (comparer(array[j], pivot) == 0))
			{
				i++;
				swap(&array[i], &array[j]);
			}
		}
		swap(&array[i + 1], &array[high]);
		return (i + 1);
	};
	if (low < high)
	{
		int pivot = partition(array, low, high);
		QuickSort(array, low, pivot - 1, comparer);
		QuickSort(array, pivot + 1, high, comparer);
	}
}

template <typename Numeric>
Numeric Average(const Numeric* start, const Numeric* end) noexcept
{
	Numeric size();
	Numeric total();
	for (unsigned i = 0; start + i != end; i++)
	{
		size++;
		total += start[i];
	}
	return total / size;
}