/*
 * linux/lib/cmdline.c
 * Helper functions generally used for parsing kernel command line
 * and module options.
 *
 * Code and copyrights come from init/main.c and arch/i386/kernel/setup.c.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 *
 * GNU Indent formatting options for this file: -kr -i8 -npsl -pcs
 *
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>

/*
 *	If a hyphen was found in get_option, this will handle the
 *	range of numbers, M-N.  This will expand the range and insert
 *	the values[M, M+1, ..., N] into the ints array in get_options.
 */

static int get_range(char **str, int *pint)
{
	int x, inc_counter, upper_range;

	(*str)++;
	upper_range = simple_strtol((*str), NULL, 0);
	inc_counter = upper_range - *pint;
	for (x = *pint; x < upper_range; x++)
		*pint++ = x;
	return inc_counter;
}

/**
 *	get_option - Parse integer from an option string
 *	@str: option string
 *	@pint: (output) integer value parsed from @str
 *
 *	Read an int from an option string; if available accept a subsequent
 *	comma as well.
 *
 *	Return values:
 *	0 - no int in string
 *	1 - int found, no subsequent comma
 *	2 - int found including a subsequent comma
 *	3 - hyphen found to denote a range
 */

int get_option(char **str, int *pint)
{
	char *cur = *str;

	if (!cur || !(*cur))
		return 0;
	*pint = simple_strtol(cur, str, 0);
	if (cur == *str)
		return 0;
	if (**str == ',') {
		(*str)++;
		return 2;
	}
	if (**str == '-')
		return 3;

	return 1;
}
EXPORT_SYMBOL(get_option);

/**
 *	get_options - Parse a string into a list of integers
 *	@str: String to be parsed
 *	@nints: size of integer array
 *	@ints: integer array
 *
 *	This function parses a string containing a comma-separated
 *	list of integers, a hyphen-separated range of _positive_ integers,
 *	or a combination of both.  The parse halts when the array is
 *	full, or when no more numbers can be retrieved from the
 *	string.
 *
 *	Return value is the character in the string which caused
 *	the parse to end (typically a null terminator, if @str is
 *	completely parseable).
 */

char *get_options(const char *str, int nints, int *ints)
{
	int res, i = 1;

	while (i < nints) {
		res = get_option((char **)&str, ints + i);
		if (res == 0)
			break;
		if (res == 3) {
			int range_nums;
			range_nums = get_range((char **)&str, ints + i);
			if (range_nums < 0)
				break;
			/*
			 * Decrement the result by one to leave out the
			 * last number in the range.  The next iteration
			 * will handle the upper number in the range
			 */
			i += (range_nums - 1);
		}
		i++;
		if (res == 1)
			break;
	}
	ints[0] = i - 1;
	return (char *)str;
}
EXPORT_SYMBOL(get_options);

/**
 *	memparse - parse a string with mem suffixes into a number
 *	@ptr: Where parse begins
 *	@retptr: (output) Optional pointer to next char after parse completes
 *
 *	Parses a string into a number.  The number stored at @ptr is
 *	potentially suffixed with K, M, G, T, P, E.
 */

unsigned long long memparse(const char *ptr, char **retptr)
{
	char *endptr;	/* local pointer to end of parsed string */

	unsigned long long ret = simple_strtoull(ptr, &endptr, 0);

	switch (*endptr) {
	case 'E':
	case 'e':
		ret <<= 10;
	case 'P':
	case 'p':
		ret <<= 10;
	case 'T':
	case 't':
		ret <<= 10;
	case 'G':
	case 'g':
		ret <<= 10;
	case 'M':
	case 'm':
		ret <<= 10;
	case 'K':
	case 'k':
		ret <<= 10;
		endptr++;
	default:
		break;
	}

	if (retptr)
		*retptr = endptr;

	return ret;
}
EXPORT_SYMBOL(memparse);

/**
 *	parse_option_str - Parse a string and check an option is set or not
 *	@str: String to be parsed
 *	@option: option name
 *
 *	This function parses a string containing a comma-separated list of
 *	strings like a=b,c.
 *
 *	Return true if there's such option in the string, or return false.
 */
bool parse_option_str(const char *str, const char *option)
{
	while (*str) {
		if (!strncmp(str, option, strlen(option))) {
			str += strlen(option);
			if (!*str || *str == ',')
				return true;
		}

		while (*str && *str != ',')
			str++;

		if (*str == ',')
			str++;
	}

	return false;
}

/**
 *	find_bootarg_content - Parse cmdline and give content info back
 *	@par_name: Parameter name to be parsed
 *	@pos: Start position of the content (within the cmdline string)
 *	@len: Length of the content
 *
 *	This function parses the cmdline and give position and length of the
 *	content back. If parameter hasn't a value it returns also true but the
 *	length will be zero.
 *
 *	Return true if par_name is found
 */
bool find_bootarg_content(const char *par_name, int *pos, int *len)
{
	int i = 0;
	int end = 0;
	int pos_value = 0;
	int len_value = 0;
	const char *cmdline = saved_command_line;

	/* Not possible */
	if (strlen(par_name) > strlen(cmdline))
		return false;

	end = strlen(cmdline) - strlen(par_name) + 1;
	for (i = 0; i < end; i++)
		if (!strncmp( cmdline + i, par_name, strlen(par_name)))
			break;

	/* Parameter name wasn't found */
	if (i == end)
		return false;

	pos_value = i + strlen(par_name);
	len_value = 0;

	/* Parameter without value */
	if ((cmdline[pos_value] == 0x20) || (cmdline[pos_value] == 0x00))
		return true;

	/* Paremeter with value: There must be a '=' after parameter name */
	if (cmdline[pos_value] != '=')
		return false;

	/* Skip character '=' */
	(pos_value)++;

	/* There must be a character other that delimiter (space) or termination */
	if ((cmdline[pos_value] == 0x20) || (cmdline[pos_value] == 0x00))
		return false;

	for (i = pos_value; i < strlen(cmdline); i++)
		if ((cmdline[i] == 0x20) || (cmdline[i] == 0x00))
			break;

	len_value = i - pos_value;

	if (pos != NULL)
		*pos = pos_value;

	if (len != NULL)
		*len = len_value;

	return true;
}

/**
 *	get_bootarg_content - Parse cmdline and give the found content back
 *	@par_name: Parameter name to be parsed
 *	@values: Pointer to an array of strings, which contain the content
 *	@size: Number of array elements
 *
 *	This function parses the cmdline and give the found content back. Each
 *	name and value combo e.g. CLK:100 (separated by comma) is stored in a
 *	single string. The number of found name and value combos were given back
.*	by size. If parameter hasn't a value it returns also true but the size
.*	will be zero.
 *
 *	Return true if par_name is found
 */
bool get_bootarg_content(const char *par_name, char ***values, int *size)
{
	const char *cmdline = saved_command_line;
	int pos = 0;
	int len = 0;
	int i = 0;
	int index = 0;
	char *par_str = NULL;
	bool ret = false;

	ret = find_bootarg_content(par_name, &pos, &len);

	/* If caller only want to check for par_name
	   Don't allocate memory, just leave here */
	if ((values == NULL) || (size == NULL))
		return ret;

	*size = 0;
	if ((ret == true) && (len != 0)) {
		par_str = kzalloc(len + 1, GFP_KERNEL); /* (+1) => termination */
		if (par_str) {
			strncpy(par_str, cmdline + pos, len);

			/* Replace separator ',' with zero (termination) */
			for (i = 0; i < len; i++)
				if (par_str[i] == ',')
					par_str[i] = '\0';

			/* Determine the amound of values */
			for (i = 0; i < len + 1; i++) /* (+1) also count last termination */
				if (par_str[i] == '\0')
					(*size)++;

			/* Separate value strings */
			*values = kzalloc((*size)*sizeof(char *), GFP_KERNEL);
			index = 0;
			(*values)[index++] = par_str;
			for (i = 0; i < len; i++)
				if (par_str[i] == '\0')
					(*values)[index++] = par_str + i + 1;
		}
	}

	return ret;
}
EXPORT_SYMBOL(get_bootarg_content);
